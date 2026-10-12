#!/usr/bin/env python3
"""Queue a targeted LoRa reboot after two missed hourly node reports."""

import json
import os
import threading
import time
from datetime import datetime

import paho.mqtt.client as mqtt


MQTT_HOST = "localhost"
MQTT_PORT = 1883
COMMAND_TOPIC = "farm/node/command"
REPORT_SECONDS = 3600
MISSED_REPORT_LIMIT = 2
LATE_GRACE_SECONDS = 300
REBOOT_AFTER_SECONDS = REPORT_SECONDS * MISSED_REPORT_LIMIT + LATE_GRACE_SECONDS
REQUEUE_AFTER_SECONDS = 6 * 3600
CHECK_SECONDS = 30
STATE_FILE = "/home/farmpi/farm/database/node_watchdog_state.json"


state_lock = threading.Lock()
state = {"nodes": {}}


def log(message):
    stamp = datetime.now().astimezone().isoformat(timespec="seconds")
    print(f"{stamp} {message}", flush=True)


def load_state():
    global state
    try:
        with open(STATE_FILE, "r", encoding="utf-8") as handle:
            loaded = json.load(handle)
        if isinstance(loaded, dict) and isinstance(loaded.get("nodes"), dict):
            state = loaded
            log(f"[STATE] Loaded {len(state['nodes'])} tracked node(s)")
            return
    except FileNotFoundError:
        pass
    except (OSError, json.JSONDecodeError) as error:
        log(f"[STATE] Could not load saved state: {error}")

    state = {"nodes": {}}
    log("[STATE] Starting with an empty node list")


def save_state_locked():
    temporary = f"{STATE_FILE}.tmp"
    try:
        with open(temporary, "w", encoding="utf-8") as handle:
            json.dump(state, handle, separators=(",", ":"), sort_keys=True)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, STATE_FILE)
    except OSError as error:
        log(f"[STATE] Save failed: {error}")


def valid_node_id(value):
    return isinstance(value, str) and (
        value.startswith("soil_") or value.startswith("tank_")
    )


def on_connect(client, userdata, flags, reason_code, properties):
    log(f"[MQTT] Connected: {reason_code}")
    client.subscribe("farm/soil/+", qos=1)
    client.subscribe("farm/tank/+", qos=1)


def on_disconnect(client, userdata, disconnect_flags, reason_code, properties):
    log(f"[MQTT] Disconnected: {reason_code}")


def on_message(client, userdata, message):
    try:
        payload = json.loads(message.payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return

    if not isinstance(payload, dict):
        return

    topic_node = message.topic.rsplit("/", 1)[-1]
    node_id = payload.get("node", topic_node)

    if not valid_node_id(node_id) or node_id != topic_node:
        log(f"[REPORT] Ignored node/topic mismatch on {message.topic}")
        return

    now = time.time()
    supports_reboot = payload.get("remote_reboot") is True

    with state_lock:
        nodes = state["nodes"]
        record = nodes.get(node_id)

        if record is None:
            # On first installation, retained MQTT reports have no original
            # timestamp. Seed them at startup and begin accurate monitoring
            # from this point rather than immediately rebooting every node.
            record = {
                "topic": message.topic,
                "last_seen": now,
                "remote_reboot": supports_reboot,
                "incident_active": False,
                "last_reboot_queued": None,
                "seeded_from_retained": bool(message.retain),
            }
            nodes[node_id] = record
            save_state_locked()
            source = "retained seed" if message.retain else "live report"
            log(f"[REPORT] Tracking {node_id} from {source}; reboot={supports_reboot}")
            return

        record["topic"] = message.topic
        record["remote_reboot"] = supports_reboot

        if message.retain:
            # A retained replay after this service restarts is old data. Keep
            # the persisted last_seen value so an offline node cannot appear
            # freshly online merely because the monitor restarted.
            save_state_locked()
            return

        previous_incident = bool(record.get("incident_active"))
        record["last_seen"] = now
        record["incident_active"] = False
        record["last_reboot_queued"] = None
        record["seeded_from_retained"] = False
        save_state_locked()

    if previous_incident:
        log(f"[RECOVERY] {node_id} reported again; incident cleared")


def queue_due_reboots(client):
    now = time.time()

    with state_lock:
        candidates = []

        for node_id, record in state["nodes"].items():
            if record.get("remote_reboot") is not True:
                continue

            try:
                age = now - float(record["last_seen"])
            except (KeyError, TypeError, ValueError):
                continue

            if age < REBOOT_AFTER_SECONDS:
                continue

            last_queued = record.get("last_reboot_queued")
            if last_queued is not None and now - float(last_queued) < REQUEUE_AFTER_SECONDS:
                continue

            candidates.append((node_id, age))

    for node_id, age in candidates:
        command = {
            "node": node_id,
            "target": node_id,
            "cmd": "reboot",
            "reason": "two_missed_reports",
        }
        encoded = json.dumps(command, separators=(",", ":"))

        try:
            result = client.publish(COMMAND_TOPIC, encoded, qos=1, retain=True)
            result.wait_for_publish(timeout=10)
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                raise RuntimeError(f"MQTT publish returned {result.rc}")
        except Exception as error:
            log(f"[REBOOT] Could not queue command for {node_id}: {error}")
            continue

        with state_lock:
            record = state["nodes"].get(node_id)
            if record is not None:
                record["incident_active"] = True
                record["last_reboot_queued"] = now
                save_state_locked()

        log(
            f"[REBOOT] Queued targeted reboot for {node_id} "
            f"after {age / 3600:.2f} hours without a report"
        )


def main():
    load_state()

    client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2,
        client_id="farm-node-watchdog",
    )
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.reconnect_delay_set(min_delay=2, max_delay=60)
    client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
    client.loop_start()

    log(
        f"[START] Monitoring capable nodes; reboot threshold "
        f"{REBOOT_AFTER_SECONDS} seconds"
    )

    try:
        while True:
            queue_due_reboots(client)
            time.sleep(CHECK_SECONDS)
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
