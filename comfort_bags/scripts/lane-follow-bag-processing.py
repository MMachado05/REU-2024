import rosbag
import csv
import sys

def extract_msg_data(msg):
    """
    Extract data from a ROS message, handling simple and nested messages.
    """
    if hasattr(msg, '__slots__'):
        msg_dict = {}
        for slot in msg.__slots__:
            val = getattr(msg, slot)
            if hasattr(val, '__slots__'):  # if the value is another ROS message
                sub_msg_dict = extract_msg_data(val)
                for sub_slot in sub_msg_dict:
                    msg_dict[f"{slot}.{sub_slot}"] = sub_msg_dict[sub_slot]
            else:
                msg_dict[slot] = val
        return msg_dict
    else:
        return {"data": msg}

def bag_to_csv(bag_file, csv_file, topics):
    bag = rosbag.Bag(bag_file)
    topic_data = {topic: [] for topic in topics}

    # Iterate through the bag and extract data
    for topic, msg, t in bag.read_messages(topics=topics):
        msg_dict = extract_msg_data(msg)
        timestamp = t.to_sec()  # Keep time as float for uniqueness
        msg_dict['time'] = timestamp
        topic_data[topic].append(msg_dict)

    bag.close()

    # Write to CSV
    with open(csv_file, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        # Write the header
        headers = ['time'] + [f"{topic}_{field}" for topic in topics for field in topic_data[topic][0].keys() if field != 'time']
        csv_writer.writerow(headers)

        # Find the longest topic data length
        max_length = max(len(topic_data[topic]) for topic in topics)

        for i in range(max_length):
            row = []
            time_set = False
            for topic in topics:
                if i < len(topic_data[topic]):
                    msg_dict = topic_data[topic][i]
                    if not time_set:
                        row.append(msg_dict['time'])
                        time_set = True
                    for field in topic_data[topic][0].keys():
                        if field != 'time':
                            row.append(msg_dict.get(field, ''))
                else:
                    if not time_set:
                        row.append('')
                        time_set = True
                    row.extend([''] * (len(topic_data[topic][0]) - 1))
            csv_writer.writerow(row)

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: python multi_topic_bag_to_csv.py <input_bag_file> <output_csv_file> <topic1> [<topic2> ... <topicN>]")
        sys.exit(1)

    bag_file = sys.argv[1]
    csv_file = sys.argv[2]
    topics = sys.argv[3:]
    bag_to_csv(bag_file, csv_file, topics)
