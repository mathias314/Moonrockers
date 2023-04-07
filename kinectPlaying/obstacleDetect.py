"""Script to take in output that was written to the disk from rostopic echo /depth/image_raw topic,
that reconstructs the image, finds the 2nd derivative, and writes a list of objects, their locations, and their distances to a file.

Will need to be modified and placed into the ros workspace, but this gives an overview of an obstacle detection algorithm.

To use, take a frame of the yaml output from rostopic echo /depth/image_raw, and use that as the command line argument for the script.
python3 obstacleDetect.py DepthTopicOutput.yaml
"""

from collections import defaultdict
import yaml
import numpy as np
import pandas as pd
import cv2
from PIL import Image as im
import dataframe_image as dfi
import struct
import time
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

VISUALIZE = True


def main():
    if len(sys.argv) != 2:
        print("Please enter yaml file to convert to image!")
        exit(1)

    fileName = sys.argv[1]
    # Load the YAML data from the file
    with open(fileName, "r") as f:
        fileName = fileName[:-4]

        data = yaml.safe_load(f)
        array = data["data"]

        print("Finished loading in yaml, now to process image!")
        start = time.time()

        packed = struct.pack(
            "<" + "H" * (len(array) // 2),
            *[array[i] + (array[i + 1] << 8) for i in range(0, len(array), 2)]
        )

        merged = struct.unpack("<" + "H" * (len(array) // 2), packed)
        # Merge the raw data from the rostopic output in little endian fashion to make a 16 bit value
        # Merges the 2 8 bit values into 1 16 bit output

        rowCounter = 0
        colCounter = 0
        imageMatrix = np.zeros((1024, 1024), dtype=np.uint16)
        picDict = defaultdict(list)
        for val in merged:
            imageMatrix[rowCounter][colCounter] = val
            picDict["x"].append(rowCounter)
            picDict["y"].append(colCounter)
            picDict["val"].append(val)

            colCounter += 1
            if colCounter == 1024:
                rowCounter += 1
                colCounter = 0

        df = pd.DataFrame(picDict)
        # pd.DataFrame.to_csv(df, fileName + ".csv")
        image = (imageMatrix).astype(
            np.uint16
        )  # To switch to 16 bit image, remove divide by 256 and change type to uint16

        cv2.imwrite(fileName + ".tiff", image)

        image = cv2.imread(fileName + ".tiff", cv2.IMREAD_GRAYSCALE)

        # Apply a Gaussian filter to the image to remove noise
        image = cv2.GaussianBlur(image, (3, 3), 1)

        # Take the first derivative of the image in the x direction
        dx = cv2.Sobel(image, cv2.CV_32F, 1, 0, 3)

        # Take the second derivative of the image in the x direction
        dxx = cv2.Sobel(dx, cv2.CV_32F, 1, 0, 3)

        # Save the image to disk
        cv2.imwrite(fileName + "dxx.png", dxx)

        derivativeArray = cv2.imread(fileName + "dxx.png", cv2.IMREAD_GRAYSCALE)
        objectMatrix = np.zeros((1024, 1024), dtype=np.uint8)
        objectDict = defaultdict(list)
        for i, row in enumerate(derivativeArray):
            for j, entry in enumerate(row):
                if entry != 0 and imageMatrix[i][j] != 0:
                    objectMatrix[i][j] = imageMatrix[i][j]
                    objectDict["x"].append(i)
                    objectDict["y"].append(j)
                    objectDict["distance"].append(imageMatrix[i][j])

        # totalImageDataFrame = pd.DataFrame(objectMatrix)
        # pd.DataFrame.to_csv(totalImageDataFrame, fileName + "Derivative.csv")

        obstacleList = pd.DataFrame(objectDict)
        pd.DataFrame.to_csv(obstacleList, fileName + "ObstacleList.csv")

        finish = time.time() - start
        print("Time to finish was " + str(finish))

        if VISUALIZE:
            createVisualization(fileName + "ObstacleList.csv")


def createVisualization(fileName: str) -> None:
    df = pd.read_csv(fileName)
    x = df["x"]
    y = df["y"]
    dist = df["distance"]

    # create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    # plot the points
    ax.scatter(x, y, dist)

    # set the axis labels
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Distance")

    # show the plot
    plt.show()


if __name__ == "__main__":
    main()
