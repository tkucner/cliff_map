#!/usr/bin/env python
import rosbag
import sys
import getopt
import csv


class BagProcess:
    def __init__(self):
        self.inputFile = ''
        self.outputFile = ''
        self.bag = None
        self.subsample = 1

    def preview(self):
        self.bag = rosbag.Bag(self.inputFile)
        topics = self.bag.get_type_and_topic_info()[1].keys()
        types = []
        for i in range(0, len(self.bag.get_type_and_topic_info()[1].values())):
            types.append(self.bag.get_type_and_topic_info()[1].values()[i][0])
            load = self.bag.read_messages(topics=[topics[i]])
            count = 0
            for topic, msg, t in load:
                count += 1
            print(topics[i], types[i], count)

    def dump(self):
        self.bag = rosbag.Bag(self.inputFile)
        topics = self.bag.get_type_and_topic_info()[1].keys()
        types = []
        for i in range(0, len(self.bag.get_type_and_topic_info()[1].values())):
            types.append(self.bag.get_type_and_topic_info()[1].values()[i][0])
            load = self.bag.read_messages(topics=[topics[i]])
            with open(self.outputFile, 'w') as csvfile:
                fieldnames = ['id', 't', 'x', 'y', 'u', 'v']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                counter = 0
                for topic, msg, t in load:
                    if counter % self.subsample == 0:
                        for j in range(0, len(msg.tracks)):
                            writer.writerow({'id': str(j),
                                             't': str(msg.header.stamp),
                                             'x': str(msg.tracks[j].pose.pose.position.x),
                                             'y': str(msg.tracks[j].pose.pose.position.y),
                                             'u': str(msg.tracks[j].twist.twist.linear.x),
                                             'v': str(msg.tracks[j].twist.twist.linear.y)})
                    counter=counter+1


def print_help():
    print('-i <input_file> - input bag-file to parse')
    print('-o <output_file> - output csv file with data')
    print('-d dump data to cvs file')
    print('-p preview whole data')
    print('-s subsample the data')


def main(argv):
    bag_process = BagProcess()
    preview = False
    dump = False
    try:
        opts, args = getopt.getopt(argv, "htpdi:o:s:")
    except getopt.GetoptError:
        print_help()
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print_help()
            sys.exit()
        if opt == '-i':
            setattr(bag_process, 'inputFile', arg)
        if opt == '-o':
            setattr(bag_process, 'outputFile', arg)
        if opt == '-p':
            preview = True
        if opt == '-s':
            setattr(bag_process, 'subsample', int(arg))
        if opt == '-d':
            dump = True
    if getattr(bag_process, 'inputFile') == '':
        print_help()
        sys.exit(2)
    if preview:
        bag_process.preview()
    if dump:
        bag_process.dump()


if __name__ == "__main__":
    main(sys.argv[1:])
