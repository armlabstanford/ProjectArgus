#!/usr/bin/env python3

import sys
from typing import List, Tuple

import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib.pyplot as plt
import rosbag2_py


def get_header_stamp(msg):
    if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
        stamp = msg.header.stamp
        return stamp.sec * 10**9 + stamp.nanosec
    return None


def apply_namespace(topic: str, namespace: str):
    """Prefix topic with namespace."""
    if not namespace:
        return topic

    # Normalize slashes
    namespace = namespace.strip("/")
    topic = topic.strip("/")

    return f"/{namespace}/{topic}"


def read_bag_messages(bag_path, namespace):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    while reader.has_next():
        topic, data, t = reader.read_next()

        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        header_stamp = get_header_stamp(msg)
        if header_stamp is None:
            header_stamp = t

        namespaced_topic = apply_namespace(topic, namespace)

        yield (header_stamp, namespaced_topic, msg, type_map[topic])

def merge_bags_with_namespaces(bag_namespace_pairs: List[Tuple[str, str]], output_bag: str):
    all_msgs = []
    bag_timestamps = []  # list of lists, one per bag

    for bag_idx, (bag, namespace) in enumerate(bag_namespace_pairs):
        print(f"Reading {bag} with namespace '{namespace}'...")
        timestamps = []

        for item in read_bag_messages(bag, namespace):
            header_stamp, topic, msg, type_str = item
            all_msgs.append(item)
            timestamps.append(header_stamp)

        bag_timestamps.append((bag_idx, bag, timestamps))

    print(f"Total messages: {len(all_msgs)}")

    # ✅ Plot timestamps
    print("Plotting timestamps...")
    plt.figure()

    for bag_idx, bag_name, timestamps in bag_timestamps:
        if not timestamps:
            continue

        # Convert to seconds relative to first timestamp in that bag
        t0 = timestamps[0]
        times_sec = [(t - t0) / 1e9 for t in timestamps]

        y_vals = [bag_idx] * len(times_sec)

        plt.scatter(times_sec, y_vals, s=2, label=f"{bag_name}")

    plt.xlabel("Time (s, relative per bag)")
    plt.ylabel("Bag Index")
    plt.title("Message Timestamps per Bag")
    plt.legend()
    plt.grid(True)

    plt.show()

    # ✅ Continue with merge
    print("Sorting by header.stamp...")
    all_msgs.sort(key=lambda x: x[0])

    storage_options = rosbag2_py.StorageOptions(uri=output_bag, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')

    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options, converter_options)

    # Register topics
    created_topics = set()
    for _, topic, _, type_str in all_msgs:
        if topic not in created_topics:
            writer.create_topic(
                rosbag2_py.TopicMetadata(
                    name=topic,
                    type=type_str,
                    serialization_format='cdr'
                )
            )
            created_topics.add(topic)

    print("Writing output bag...")
    for header_stamp, topic, msg, type_str in all_msgs:
        writer.write(topic, serialize_message(msg), header_stamp)

    print(f"Done. Output bag: {output_bag}")

if __name__ == "__main__":
    if len(sys.argv) < 4 or len(sys.argv[2:]) % 2 != 0:
        print("Usage:")
        print("  merge_bags.py output_bag bag1 namespace1 [bag2 namespace2 ...]")
        sys.exit(1)

    output_bag = sys.argv[1]

    args = sys.argv[2:]
    bag_namespace_pairs = []

    for i in range(0, len(args), 2):
        bag = args[i]
        namespace = args[i + 1]
        bag_namespace_pairs.append((bag, namespace))

    rclpy.init()
    merge_bags_with_namespaces(bag_namespace_pairs, output_bag)
    rclpy.shutdown()