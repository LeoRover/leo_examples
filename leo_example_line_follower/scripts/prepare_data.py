#!/usr/bin/env python
import os
from pathlib import Path
import pickle
import sys
import argparse
import shutil
from zipfile import ZipFile


class DataProcessor:
    def __init__(self, train, valid, zip_file):
        self.training_paths = train
        self.validation_paths = valid
        self.zip_file = zip_file

        if not zip_file.endswith(".zip"):
            self.zip_file += ".zip"

        try:
            Path("data").mkdir(parents=True)
        except FileExistsError as e:
            print(
                "'data' directory exist in the working directory already. Can't process images."
            )
            return

        self.process_data(self.training_paths + self.validation_paths)
        self.make_zip_archive()

    def make_zip_archive(self):
        zipObj = None
        try:
            zipObj = ZipFile(self.zip_file, "x")
        except FileExistsError as e:
            print(
                "Given zip file already exist. Pick different name, or move the zip file."
            )
            shutil.rmtree("data")
            os.remove("partition.pickle")
            os.remove("labels.pickle")
            return

        print("Created %s file" % (self.zip_file))

        zipObj.write("partition.pickle")
        print("Zipped 'partition.pickle' file.")
        zipObj.write("labels.pickle")
        print("Zipped 'labels.pickle' file.")

        images = os.listdir("data")

        for img in images:
            zipObj.write(os.path.join("data", img))
        print("Zipped 'data' directory.")
        zipObj.close()

        print("Zip archive ready, removing temp files.")
        shutil.rmtree("data")
        os.remove("partition.pickle")
        os.remove("labels.pickle")

    def process_data(self, paths):
        labels = {"linear": {}, "angular": {}}
        partition = {"validation": [], "train": []}

        for path in paths:
            print("processing %s/labels.txt" % (path))
            file = open(os.path.join(path, "labels.txt"), "r")
            lines = file.readlines()

            for line in lines:
                photo, label_tuple = self.get_label(line)
                labels["linear"][photo] = label_tuple[0]
                labels["angular"][photo] = label_tuple[1]

                if path in self.validation_paths:
                    partition["validation"].append(photo)
                else:
                    partition["train"].append(photo)

                shutil.copy(os.path.join(path, photo), os.path.join("data", photo))
            file.close()

        with open("labels.pickle", "wb") as handle:
            pickle.dump(labels, handle, pickle.DEFAULT_PROTOCOL)

        with open("partition.pickle", "wb") as handle:
            pickle.dump(partition, handle, protocol=pickle.DEFAULT_PROTOCOL)

    def make_tuple(self, string):
        divide = string.split(",")
        first = float(divide[0][1:])
        second = float(divide[1][:-1])
        return (first, second)

    def get_label(self, line):
        divide = line.split(":")
        key = divide[0]
        value = self.make_tuple(divide[1].strip())
        return key, value


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Prepare dataset for neural network")
    parser.add_argument(
        "-t",
        "--train_data",
        nargs="+",
        type=str,
        required=True,
        metavar="[paths]",
        help="paths to directiories with training data",
        dest="train",
    )
    parser.add_argument(
        "-v",
        "--valid_data",
        nargs="+",
        type=str,
        required=True,
        metavar="[paths]",
        help="paths to directiories with validation data",
        dest="valid",
    )
    parser.add_argument(
        "-z",
        "--zip_file",
        nargs="?",
        type=str,
        metavar="path",
        help="name of the zip archive with dataset",
        dest="zip",
        default="my_dataset.zip",
    )

    args = parser.parse_args(sys.argv[1:])

    data_processor = DataProcessor(args.train, args.valid, args.zip)
