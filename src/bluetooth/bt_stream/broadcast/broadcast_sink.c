/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "broadcast_sink.h"

#include <zephyr/zbus/zbus.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/pacs.h>
#include <zephyr/bluetooth/audio/csip.h>
#include <zephyr/bluetooth/audio/cap.h>
#include <zephyr/sys/byteorder.h>

/* TODO: Remove when a get_info function is implemented in host */
#include <../subsys/bluetooth/audio/bap_endpoint.h>

#include "bt_mgmt.h"
#include "macros_common.h"
#include "zbus_common.h"
#include "channel_assignment.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(broadcast_sink, 4);



ZBUS_CHAN_DEFINE(le_audio_chan, struct le_audio_msg, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0));

static bool broadcast_code_received = false;
static uint8_t bis_encryption_key[BT_ISO_BROADCAST_CODE_SIZE] = {0};

struct audio_codec_info {
	uint8_t id;
	uint16_t cid;
	uint16_t vid;
	int frequency;
	int frame_duration_us;
	enum bt_audio_location chan_allocation;
	int octets_per_sdu;
	int bitrate;
	int blocks_per_sdu;
};
struct active_audio_stream {
	struct bt_bap_stream *stream;
	struct audio_codec_info *codec;
	uint32_t pd;
};

static struct bt_bap_broadcast_sink *broadcast_sink;
static struct bt_bap_stream audio_streams[CONFIG_BT_BAP_BROADCAST_SNK_STREAM_COUNT];
static struct audio_codec_info audio_codec_info[CONFIG_BT_BAP_BROADCAST_SNK_STREAM_COUNT];
static uint32_t bis_index_bitfields[CONFIG_BT_BAP_BROADCAST_SNK_STREAM_COUNT];
static struct bt_le_per_adv_sync *pa_sync_stored;
static struct active_audio_stream active_stream;

#define MAX_SUBGROUPS CONFIG_BT_BAP_BROADCAST_SNK_SUBGROUP_COUNT
#define MAX_BISES_PER_SUBGROUP CONFIG_BT_BAP_BROADCAST_SNK_STREAM_COUNT

struct parsed_subgroup {
	uint8_t bis_indices[CONFIG_BT_BAP_BROADCAST_SNK_STREAM_COUNT];
	uint8_t bis_count;
	struct audio_codec_info codec_info;
};

static struct parsed_subgroup parsed_subgroups[MAX_SUBGROUPS];
static uint8_t subgroup_total_count;
static uint8_t current_subgroup_index;

/* The values of sync_stream_cnt and active_stream_index must never become larger
 * than the sizes of the arrays above (audio_streams etc.)
 */
static uint8_t sync_stream_cnt;
static uint8_t active_stream_index;

static struct bt_audio_codec_cap codec_cap = BT_AUDIO_CODEC_CAP_LC3(
	BT_AUDIO_CODEC_CAPABILIY_FREQ,
	(BT_AUDIO_CODEC_CAP_DURATION_10 | BT_AUDIO_CODEC_CAP_DURATION_PREFER_10),
	BT_AUDIO_CODEC_CAP_CHAN_COUNT_SUPPORT(1), LE_AUDIO_SDU_SIZE_OCTETS(CONFIG_LC3_BITRATE_MIN),
	LE_AUDIO_SDU_SIZE_OCTETS(CONFIG_LC3_BITRATE_MAX), 1u, BT_AUDIO_CONTEXT_TYPE_ANY);

static struct bt_pacs_cap capabilities = {
	.codec_cap = &codec_cap,
};

#define AVAILABLE_SINK_CONTEXT (BT_AUDIO_CONTEXT_TYPE_ANY)

static le_audio_receive_cb receive_cb;

static bool init_routine_completed;
static bool paused;

static struct bt_csip_set_member_svc_inst *csip;

static uint8_t flags_adv_data;
static uint8_t bass_service_uuid[BT_UUID_SIZE_16];
static uint8_t gap_appear_adv_data[BT_UUID_SIZE_16];
static uint8_t csip_rsi_adv_data[BT_CSIP_RSI_SIZE];

static bool store_bis_cb(const struct bt_bap_base_subgroup_bis *bis, void *user_data)
{
    struct parsed_subgroup *ps = (struct parsed_subgroup *)user_data;

    if (ps->bis_count >= MAX_BISES_PER_SUBGROUP) {
        return false; /* we already have max BISes; stop iterating */
    }

    ps->bis_indices[ps->bis_count++] = bis->index;
    return true; /* continue if there are more BISes */
}

#define CSIP_SET_SIZE 2
enum csip_set_rank {
	CSIP_HL_RANK = 1,
	CSIP_HR_RANK = 2
};

/* Callback for locking state change from server side */
static void csip_lock_changed_cb(struct bt_conn *conn, struct bt_csip_set_member_svc_inst *csip,
				 bool locked)
{
	LOG_DBG("Client %p %s the lock", (void *)conn, locked ? "locked" : "released");
}

/* Callback for SIRK read request from peer side */
static uint8_t sirk_read_req_cb(struct bt_conn *conn, struct bt_csip_set_member_svc_inst *csip)
{
	/* Accept the request to read the SIRK, but return encrypted SIRK instead of plaintext */
	return BT_CSIP_READ_SIRK_REQ_RSP_ACCEPT_ENC;
}

static struct bt_csip_set_member_cb csip_callbacks = {
	.lock_changed = csip_lock_changed_cb,
	.sirk_read_req = sirk_read_req_cb,
};

struct bt_csip_set_member_register_param csip_param = {
	.set_size = CSIP_SET_SIZE,
	.lockable = true,
	.cb = &csip_callbacks,
};


int broadcast_sink_uuid_populate(struct net_buf_simple *uuid_buf)
{
	if (net_buf_simple_tailroom(uuid_buf) >= (BT_UUID_SIZE_16 * 3)) {
		net_buf_simple_add_le16(uuid_buf, BT_UUID_BASS_VAL);
		net_buf_simple_add_le16(uuid_buf, BT_UUID_PACS_VAL);
	} else {
		LOG_ERR("Not enough space for UUIDS");
		return -ENOMEM;
	}

	return 0;
}

int broadcast_sink_adv_populate(struct bt_data *adv_buf, uint8_t adv_buf_vacant)
{
	int ret;
	uint32_t adv_buf_cnt = 0;

	if (IS_ENABLED(CONFIG_BT_CSIP_SET_MEMBER)) {
		ret = bt_mgmt_adv_buffer_put(adv_buf, &adv_buf_cnt, adv_buf_vacant,
					     sizeof(csip_rsi_adv_data), BT_DATA_CSIS_RSI,
					     (void *)csip_rsi_adv_data);
		if (ret) {
			return ret;
		}
	}

	/*
	 * AD format required for broadcast sink with scan delegator.
	 * Details can be found in Basic Audio Profile Section 3.9.2.
	 */
	sys_put_le16(BT_UUID_BASS_VAL, &bass_service_uuid[0]);

	ret = bt_mgmt_adv_buffer_put(adv_buf, &adv_buf_cnt, adv_buf_vacant,
				     sizeof(bass_service_uuid), BT_DATA_SVC_DATA16,
				     (void *)bass_service_uuid);
	if (ret) {
		return ret;
	}

	sys_put_le16(CONFIG_BT_DEVICE_APPEARANCE, &gap_appear_adv_data[0]);

	ret = bt_mgmt_adv_buffer_put(adv_buf, &adv_buf_cnt, adv_buf_vacant,
				     sizeof(gap_appear_adv_data), BT_DATA_GAP_APPEARANCE,
				     (void *)gap_appear_adv_data);
	if (ret) {
		return ret;
	}

	flags_adv_data = BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR;

	ret = bt_mgmt_adv_buffer_put(adv_buf, &adv_buf_cnt, adv_buf_vacant, sizeof(uint8_t),
				     BT_DATA_FLAGS, (void *)&flags_adv_data);
	if (ret) {
		return ret;
	}

	return adv_buf_cnt;
}

static int broadcast_sink_cleanup(void)
{
	int ret;

	init_routine_completed = false;

	active_stream.pd = 0;
	active_stream.stream = NULL;
	active_stream.codec = NULL;

	if (broadcast_sink != NULL) {
		ret = bt_bap_broadcast_sink_delete(broadcast_sink);
		if (ret && ret != -EALREADY) {
			return ret;
		}

		broadcast_sink = NULL;
	}

	return 0;
}

static void bis_cleanup_worker(struct k_work *work)
{
	int ret;

	ret = broadcast_sink_cleanup();
	if (ret) {
		LOG_WRN("Failed to clean up BISes: %d", ret);
	}
}

K_WORK_DEFINE(bis_cleanup_work, bis_cleanup_worker);

static void le_audio_event_publish(enum le_audio_evt_type event)
{
	int ret;
	struct le_audio_msg msg;

	if (event == LE_AUDIO_EVT_SYNC_LOST) {
		msg.pa_sync = pa_sync_stored;
		pa_sync_stored = NULL;
	}

	msg.event = event;

	ret = zbus_chan_pub(&le_audio_chan, &msg, LE_AUDIO_ZBUS_EVENT_WAIT_TIME);
	ERR_CHK(ret);
}

static void print_codec(const struct audio_codec_info *codec)
{
	LOG_INF("Codec config for LC3:");
	LOG_INF("\tFrequency: %d Hz", codec->frequency);
	LOG_INF("\tFrame Duration: %d us", codec->frame_duration_us);
	LOG_INF("\tOctets per frame: %d (%d kbps)", codec->octets_per_sdu, codec->bitrate);
	LOG_INF("\tFrames per SDU: %d", codec->blocks_per_sdu);
	if (codec->chan_allocation >= 0) {
		LOG_INF("\tChannel allocation: 0x%x", codec->chan_allocation);
	}
}

static void get_codec_info(const struct bt_audio_codec_cfg *codec,
			   struct audio_codec_info *codec_info)
{
	int ret;

	ret = le_audio_freq_hz_get(codec, &codec_info->frequency);
	if (ret) {
		LOG_DBG("Failed retrieving sampling frequency: %d", ret);
	}

	ret = le_audio_duration_us_get(codec, &codec_info->frame_duration_us);
	if (ret) {
		LOG_DBG("Failed retrieving frame duration: %d", ret);
	}

	ret = bt_audio_codec_cfg_get_chan_allocation(codec, &codec_info->chan_allocation, false);
	if (ret == -ENODATA) {
		/* Codec channel allocation not set, defaulting to 0 */
		codec_info->chan_allocation = 0;
	} else if (ret) {
		LOG_DBG("Failed retrieving channel allocation: %d", ret);
	}

	ret = le_audio_octets_per_frame_get(codec, &codec_info->octets_per_sdu);
	if (ret) {
		LOG_DBG("Failed retrieving octets per frame: %d", ret);
	}

	ret = le_audio_bitrate_get(codec, &codec_info->bitrate);
	if (ret) {
		LOG_DBG("Failed calculating bitrate: %d", ret);
	}

	ret = le_audio_frame_blocks_per_sdu_get(codec, &codec_info->blocks_per_sdu);
	if (codec_info->octets_per_sdu < 0) {
		LOG_DBG("Failed retrieving frame blocks per SDU: %d", codec_info->octets_per_sdu);
	}
}

static void stream_started_cb(struct bt_bap_stream *stream)
{
	le_audio_event_publish(LE_AUDIO_EVT_STREAMING);

	/* NOTE: The string below is used by the Nordic CI system */
	LOG_INF("Stream index %d started", active_stream_index);
	print_codec(&audio_codec_info[active_stream_index]);
}

static void stream_stopped_cb(struct bt_bap_stream *stream, uint8_t reason)
{

	switch (reason) {
	case BT_HCI_ERR_LOCALHOST_TERM_CONN:
		LOG_INF("Stream stopped by user");
		le_audio_event_publish(LE_AUDIO_EVT_NOT_STREAMING);

		break;

	case BT_HCI_ERR_CONN_FAIL_TO_ESTAB:
		/* Fall-through */
	case BT_HCI_ERR_CONN_TIMEOUT:
		LOG_INF("Stream sync lost");
		k_work_submit(&bis_cleanup_work);

		le_audio_event_publish(LE_AUDIO_EVT_SYNC_LOST);

		break;

	case BT_HCI_ERR_REMOTE_USER_TERM_CONN:
		LOG_INF("Broadcast source stopped streaming");
		le_audio_event_publish(LE_AUDIO_EVT_NOT_STREAMING);

		break;

	case BT_HCI_ERR_TERM_DUE_TO_MIC_FAIL:
		LOG_INF("MIC fail. The encryption key may be wrong");
		break;

	default:
		LOG_WRN("Unhandled reason: %d", reason);

		break;
	}

	/* NOTE: The string below is used by the Nordic CI system */
	LOG_INF("Stream index %d stopped. Reason: %d", active_stream_index, reason);
}

static void stream_recv_cb(struct bt_bap_stream *stream, const struct bt_iso_recv_info *info,
			   struct net_buf *buf)
{
	bool bad_frame = false;

	if (receive_cb == NULL) {
		LOG_ERR("The RX callback has not been set");
		return;
	}

	if (!(info->flags & BT_ISO_FLAGS_VALID)) {
		bad_frame = true;
	}

	receive_cb(buf->data, buf->len, bad_frame, info->ts, active_stream_index,
		   active_stream.codec->octets_per_sdu);
}

static struct bt_bap_stream_ops stream_ops = {
	.started = stream_started_cb,
	.stopped = stream_stopped_cb,
	.recv = stream_recv_cb,
};


static bool base_subgroup_cb(const struct bt_bap_base_subgroup *subgroup, void *user_data)
{
    int ret;
    struct bt_audio_codec_cfg     codec_cfg = {0};
    struct bt_bap_base_codec_id   codec_id;

    /* If we’ve already reached our maximum supported subgroups, skip storing */
    if (subgroup_total_count >= MAX_SUBGROUPS) {
        LOG_WRN("Too many subgroups; skipping further storage (max=%d)", MAX_SUBGROUPS);
        return true; /* Continue iterating, but do nothing more for this subgroup */
    }

    /* Convert subgroup‐level codec data into a codec_cfg object */
    ret = bt_bap_base_subgroup_codec_to_codec_cfg(subgroup, &codec_cfg);
    if (ret) {
        LOG_WRN("Failed to convert codec for subgroup %d: %d", subgroup_total_count, ret);
        return true; /* skip this subgroup */
    }

    // /* Verify it’s LC3 (CID == BT_HCI_CODING_FORMAT_LC3) */
    // ret = bt_bap_base_get_subgroup_codec_id(subgroup, &codec_id);
    // if (ret || codec_id.cid != BT_HCI_CODING_FORMAT_LC3) {
    //     LOG_WRN("Unsupported codec or failed get_subgroup_codec_id: %d", ret);
    //     return true;
    // }

    /* Store “subgroup‐level” codec info in parsed_subgroups[] */
    struct parsed_subgroup *ps = &parsed_subgroups[subgroup_total_count];
    memset(ps, 0, sizeof(*ps));
    get_codec_info(&codec_cfg, &ps->codec_info);

    /* Walk through each BIS in this subgroup, storing its index via store_bis_cb() */
    ret = bt_bap_base_subgroup_foreach_bis(subgroup, store_bis_cb, ps);
    if (ret < 0) {
        LOG_WRN("Could not parse BISes for subgroup %d: %d", subgroup_total_count, ret);
    }

    LOG_INF("Stored subgroup %d (BIS count = %d)", subgroup_total_count, ps->bis_count);
    subgroup_total_count++;
    return true;
}



static void base_recv_cb(struct bt_bap_broadcast_sink *sink,
                         const struct bt_bap_base *base,
                         size_t base_size)
{
    int ret;
    bool suitable_stream_found = false;

    if (init_routine_completed) {
        return;
    }

    /* Reset parsed‐subgroup storage */
    subgroup_total_count   = 0;
    current_subgroup_index = 0;

    /* Parse all subgroups (calls base_subgroup_cb for each one) */
    ret = bt_bap_base_foreach_subgroup(base, base_subgroup_cb, &suitable_stream_found);
    if (ret != 0 && ret != -ECANCELED) {
        LOG_WRN("Failed to parse subgroups: %d", ret);
        return;
    }

    /* If no subgroups found, abort */
    if (subgroup_total_count == 0) {
        LOG_DBG("Found no subgroups in BASE");
        le_audio_event_publish(LE_AUDIO_EVT_NO_VALID_CFG);
        return;
    }

    /* We have ≥1 subgroup. Use subgroup 0 as our initial active set:
     *   - fill in sync_stream_cnt
     *   - pick first BIS index of subgroup 0
     *   - build bis_index_bitfields[] from subgroup‐0.bis_indices[]
     *   - copy subgroup‐0.codec_info → audio_codec_info[z]
     */
    struct parsed_subgroup *ps0 = &parsed_subgroups[0];

    if (ps0->bis_count == 0) {
        LOG_DBG("Subgroup 0 has no BIS—no valid streams");
        le_audio_event_publish(LE_AUDIO_EVT_NO_VALID_CFG);
        return;
    }

    /* 1. How many BIS in subgroup 0? */
    sync_stream_cnt = ps0->bis_count;

    /* 2. Clear out any old BIS bitfields & audio_codec_info */
    memset(bis_index_bitfields, 0, sizeof(bis_index_bitfields));
    // (We will also overwrite audio_codec_info only for the slots we actually need,
    //  so no need to memset(audio_codec_info).)

    /* 3. Rebuild bitfields + copy codec_info for each BIS index in subgroup 0 */
    for (int i = 0; i < ps0->bis_count; i++) {
        uint8_t one_based = ps0->bis_indices[i];
        int z = one_based - 1; /* zero‐based array index */

        /* Mark that BIS in our bitfield */
        bis_index_bitfields[z] = (1u << z);

        /* Copy codec parameters into audio_codec_info[z] */
        audio_codec_info[z] = ps0->codec_info;
    }

    /* 4. Now pick the “first BIS” from subgroup 0 → active_stream_index */
    active_stream_index = ps0->bis_indices[0] - 1;
    if ((size_t)active_stream_index >= CONFIG_BT_BAP_BROADCAST_SNK_STREAM_COUNT) {
        LOG_WRN("Initial BIS index %d out of range—clamping to 0", ps0->bis_indices[0]);
        active_stream_index = 0;
    }

    /* 5. Point active_stream to that array slot */
    active_stream.stream = &audio_streams[active_stream_index];
    active_stream.codec  = &audio_codec_info[active_stream_index];

    /* 6. Save this subgroup’s presentation delay (PD) */
    ret = bt_bap_base_get_pres_delay(base);
    if (ret < 0) {
        LOG_WRN("Failed to get pres_delay: %d", ret);
        active_stream.pd = 0;
    } else {
        active_stream.pd = ret;
    }

    le_audio_event_publish(LE_AUDIO_EVT_CONFIG_RECEIVED);
    LOG_DBG("Initialized active_stream from subgroup 0, BIS index %d",
            ps0->bis_indices[0]);
}


static void syncable_cb(struct bt_bap_broadcast_sink *sink, const struct bt_iso_biginfo *biginfo)
{
	int ret;
	struct bt_bap_stream *audio_streams_p[] = {&audio_streams[active_stream_index]};
	static uint32_t prev_broadcast_id;

	LOG_DBG("Broadcast sink is syncable");

	if (active_stream.stream != NULL && active_stream.stream->ep != NULL) {
		if (active_stream.stream->ep->status.state == BT_BAP_EP_STATE_STREAMING) {
			LOG_WRN("Syncable received, but already in a stream");
			return;
		}
	}

	if (paused) {
		LOG_DBG("Syncable received, but in paused state");
		return;
	}

	if (bis_index_bitfields[active_stream_index] == 0) {
		LOG_ERR("No bits set in bitfield");
		return;
	} else if (!IS_POWER_OF_TWO(bis_index_bitfields[active_stream_index])) {
		/* Check that only one bit is set */
		LOG_ERR("Application syncs to only one stream");
		return;
	}

	/* NOTE: The string below is used by the Nordic CI system */
	LOG_INF("Syncing to broadcast stream index %d", active_stream_index);

	if (biginfo->encryption) {
		if (!broadcast_code_received) {
			LOG_WRN("Encrypted stream: waiting for broadcast code (button press)");
			return;
		}
	
		memcpy(bis_encryption_key,
			   CONFIG_BT_AUDIO_BROADCAST_ENCRYPTION_KEY,
			   MIN(strlen(CONFIG_BT_AUDIO_BROADCAST_ENCRYPTION_KEY),
				   ARRAY_SIZE(bis_encryption_key)));
	} else {
		memset(bis_encryption_key, 0, sizeof(bis_encryption_key));
	}
	

	ret = bt_bap_broadcast_sink_sync(broadcast_sink, bis_index_bitfields[active_stream_index],
					 audio_streams_p, bis_encryption_key);

	if (ret) {
		LOG_WRN("Unable to sync to broadcast source, ret: %d", ret);
		return;
	}

	prev_broadcast_id = sink->broadcast_id;

	/* Only a single stream used for now */
	active_stream.stream = &audio_streams[active_stream_index];

	init_routine_completed = true;
}

static struct bt_bap_broadcast_sink_cb broadcast_sink_cbs = {
	.base_recv = base_recv_cb,
	.syncable = syncable_cb,
};

int broadcast_sink_change_active_audio_stream(void)
{
    int ret;

    if (broadcast_sink == NULL) {
        LOG_WRN("No broadcast sink");
        return -ECANCELED;
    }

    /* Stop stream first if needed */
    if (active_stream.stream &&
        active_stream.stream->ep &&
        active_stream.stream->ep->status.state == BT_BAP_EP_STATE_STREAMING) {
        ret = bt_bap_broadcast_sink_stop(broadcast_sink);
        if (ret) {
            LOG_ERR("Failed to stop sink before BIS switch: %d", ret);
            return ret;
        }
    }

    /* Cycle within subgroup */
    if (++active_stream_index >= sync_stream_cnt) {
        active_stream_index = 0;
    }

    active_stream.stream = &audio_streams[active_stream_index];
    active_stream.codec  = &audio_codec_info[active_stream_index];

    uint32_t bis_mask = bis_index_bitfields[active_stream_index];
    struct bt_bap_stream *streams[] = { &audio_streams[active_stream_index] };

    ret = bt_bap_broadcast_sink_sync(broadcast_sink, bis_mask, streams, bis_encryption_key);
    if (ret) {
        LOG_ERR("Failed to sync to new BIS: %d", ret);
        return ret;
    }

    LOG_INF("Changed to BIS stream %d", active_stream_index);
    return 0;
}

int broadcast_sink_change_subgroup(void)
{
    int ret;

    if (broadcast_sink == NULL) {
        LOG_WRN("No broadcast sink");
        return -ECANCELED;
    }
    if (subgroup_total_count < 2) {
        LOG_WRN("Only %d subgroup(s) available, cannot switch", subgroup_total_count);
        return -EINVAL;
    }

    if (active_stream.stream &&
        active_stream.stream->ep &&
        active_stream.stream->ep->status.state == BT_BAP_EP_STATE_STREAMING) {
        ret = bt_bap_broadcast_sink_stop(broadcast_sink);
        if (ret) {
            LOG_ERR("Failed to stop BIS before subgroup switch: %d", ret);
            return ret;
        }
    }

    current_subgroup_index = (current_subgroup_index + 1) % subgroup_total_count;
    const struct parsed_subgroup *ps = &parsed_subgroups[current_subgroup_index];

    if (ps->bis_count == 0) {
        LOG_WRN("Selected subgroup %d has no BIS—cannot switch", current_subgroup_index);
        return -EINVAL;
    }

    /* Rebuild BIS bitfields */
    memset(bis_index_bitfields, 0, sizeof(bis_index_bitfields));
    for (int i = 0; i < ps->bis_count; i++) {
        int z = ps->bis_indices[i] - 1;
        bis_index_bitfields[z] = (1u << z);
        audio_codec_info[z] = ps->codec_info;
    }

    /* Set active stream to first BIS of new subgroup */
    sync_stream_cnt     = ps->bis_count;
    active_stream_index = ps->bis_indices[0] - 1;

    active_stream.stream = &audio_streams[active_stream_index];
    active_stream.codec  = &audio_codec_info[active_stream_index];

    /* Re-sync to broadcast source with new BIS bitfield */
    struct bt_bap_stream *audio_streams_p[] = { &audio_streams[active_stream_index] };

    ret = bt_bap_broadcast_sink_sync(broadcast_sink, bis_index_bitfields[active_stream_index],
                                     audio_streams_p, bis_encryption_key);
    if (ret) {
        LOG_ERR("Failed to re-sync to new subgroup: %d", ret);
        return ret;
    }

    LOG_INF("Switched to subgroup %d (first BIS %d → stream %d)",
            current_subgroup_index,
            ps->bis_indices[0],
            active_stream_index);

    return 0;
}

int broadcast_sink_config_get(uint32_t *bitrate, uint32_t *sampling_rate, uint32_t *pres_delay)
{
	if (active_stream.codec == NULL) {
		LOG_WRN("No active stream to get config from");
		return -ENXIO;
	}

	if (bitrate == NULL && sampling_rate == NULL && pres_delay == NULL) {
		LOG_ERR("No valid pointers received");
		return -ENXIO;
	}

	if (sampling_rate != NULL) {
		*sampling_rate = active_stream.codec->frequency;
	}

	if (bitrate != NULL) {
		*bitrate = active_stream.codec->bitrate;
	}

	if (pres_delay != NULL) {
		if (active_stream.stream == NULL) {
			LOG_WRN("No active stream");
			return -ENXIO;
		}

		*pres_delay = active_stream.pd;
	}

	return 0;
}

int broadcast_sink_pa_sync_set(struct bt_le_per_adv_sync *pa_sync, uint32_t broadcast_id)
{
	int ret;

	if (pa_sync == NULL) {
		LOG_ERR("Invalid PA sync received");
		return -EINVAL;
	}

	LOG_DBG("Trying to set PA sync with ID: %d", broadcast_id);

	if (active_stream.stream != NULL && active_stream.stream->ep != NULL) {
		if (active_stream.stream->ep->status.state == BT_BAP_EP_STATE_STREAMING) {
			ret = bt_bap_broadcast_sink_stop(broadcast_sink);
			if (ret) {
				LOG_ERR("Failed to stop broadcast sink: %d", ret);
				return ret;
			}

			broadcast_sink_cleanup();
		}
	}

	/* If broadcast_sink was not in an active stream we still need to clean it up */
	if (broadcast_sink != NULL) {
		broadcast_sink_cleanup();
	}

	ret = bt_bap_broadcast_sink_create(pa_sync, broadcast_id, &broadcast_sink);
	if (ret) {
		LOG_WRN("Failed to create sink: %d", ret);
		return ret;
	}

	pa_sync_stored = pa_sync;

	return 0;
}

int broadcast_sink_broadcast_code_set(uint8_t *broadcast_code)
{
	if (broadcast_code == NULL) {
		LOG_ERR("Invalid broadcast code received");
		return -EINVAL;
	}

	memcpy(bis_encryption_key, broadcast_code, BT_ISO_BROADCAST_CODE_SIZE);
	broadcast_code_received = true;

	return 0;
}

int broadcast_sink_start(void)
{
	if (!paused) {
		LOG_WRN("Already playing");
		return -EALREADY;
	}

	paused = false;
	return 0;
}

int broadcast_sink_stop(void)
{
	int ret;

	if (paused) {
		LOG_WRN("Already paused");
		return -EALREADY;
	}

	if (active_stream.stream == NULL || active_stream.stream->ep == NULL) {
		LOG_WRN("Stream or endpoint not set");
		return -EPERM;
	}

	if (active_stream.stream->ep->status.state == BT_BAP_EP_STATE_STREAMING) {
		paused = true;
		ret = bt_bap_broadcast_sink_stop(broadcast_sink);
		if (ret) {
			LOG_ERR("Failed to stop broadcast sink: %d", ret);
			return ret;
		}
	} else {
		LOG_WRN("Current stream not in streaming state");
		return -EALREADY;
	}

	return 0;
}

int broadcast_sink_disable(void)
{
	int ret;

	if (active_stream.stream != NULL && active_stream.stream->ep != NULL) {
		if (active_stream.stream->ep->status.state == BT_BAP_EP_STATE_STREAMING) {
			ret = bt_bap_broadcast_sink_stop(broadcast_sink);
			if (ret) {
				LOG_ERR("Failed to stop sink");
			}
		}
	}

	if (pa_sync_stored != NULL) {
		ret = bt_le_per_adv_sync_delete(pa_sync_stored);
		if (ret) {
			LOG_ERR("Failed to delete pa_sync");
			return ret;
		}
	}

	ret = broadcast_sink_cleanup();
	if (ret) {
		LOG_ERR("Error cleaning up");
		return ret;
	}

	LOG_DBG("Broadcast sink disabled");

	return 0;
}

int broadcast_sink_enable(le_audio_receive_cb recv_cb)
{
	int ret;
	static bool initialized;
	enum audio_channel channel;

	if (initialized) {
		LOG_WRN("Already initialized");
		return -EALREADY;
	}

	if (recv_cb == NULL) {
		LOG_ERR("Receive callback is NULL");
		return -EINVAL;
	}

	receive_cb = recv_cb;

	channel_assignment_get(&channel);

	if (channel == AUDIO_CH_L) {
		ret = bt_pacs_set_location(BT_AUDIO_DIR_SINK, BT_AUDIO_LOCATION_FRONT_LEFT);
		csip_param.rank = CSIP_HL_RANK;
	} else {
		ret = bt_pacs_set_location(BT_AUDIO_DIR_SINK, BT_AUDIO_LOCATION_FRONT_RIGHT);
		csip_param.rank = CSIP_HR_RANK;
	}

	if (ret) {
		LOG_ERR("Location set failed");
		return ret;
	}

	ret = bt_pacs_set_supported_contexts(BT_AUDIO_DIR_SINK, AVAILABLE_SINK_CONTEXT);
	if (ret) {
		LOG_ERR("Supported context set failed. Err: %d", ret);
		return ret;
	}

	ret = bt_pacs_set_available_contexts(BT_AUDIO_DIR_SINK, AVAILABLE_SINK_CONTEXT);
	if (ret) {
		LOG_ERR("Available context set failed. Err: %d", ret);
		return ret;
	}

	ret = bt_pacs_cap_register(BT_AUDIO_DIR_SINK, &capabilities);
	if (ret) {
		LOG_ERR("Capability register failed (ret %d)", ret);
		return ret;
	}

	if (IS_ENABLED(CONFIG_BT_AUDIO_SCAN_DELEGATOR)) {
		if (IS_ENABLED(CONFIG_BT_CSIP_SET_MEMBER_TEST_SAMPLE_DATA)) {
			LOG_WRN("CSIP test sample data is used, must be changed "
				"before production");
		} else {
			if (strcmp(CONFIG_BT_SET_IDENTITY_RESOLVING_KEY_DEFAULT,
				   CONFIG_BT_SET_IDENTITY_RESOLVING_KEY) == 0) {
				LOG_WRN("CSIP using the default SIRK, must be changed "
					"before production");
			}
			memcpy(csip_param.sirk, CONFIG_BT_SET_IDENTITY_RESOLVING_KEY,
			       BT_CSIP_SIRK_SIZE);
		}

		ret = bt_cap_acceptor_register(&csip_param, &csip);
		if (ret) {
			LOG_ERR("Failed to register CAP acceptor. Err: %d", ret);
			return ret;
		}

		ret = bt_csip_set_member_generate_rsi(csip, csip_rsi_adv_data);
		if (ret) {
			LOG_ERR("Failed to generate RSI. Err: %d", ret);
			return ret;
		}
	}

	bt_bap_broadcast_sink_register_cb(&broadcast_sink_cbs);

	for (int i = 0; i < ARRAY_SIZE(audio_streams); i++) {
		audio_streams[i].ops = &stream_ops;
	}

	initialized = true;

	LOG_DBG("Broadcast sink enabled");

	return 0;
}
