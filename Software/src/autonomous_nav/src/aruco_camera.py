import argparse
import sys
import cv2
import imutils


# define names of each possible ArUco tag OpenCV supports
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}


def main(aruco_dict):
    video_capture = cv2.VideoCapture(2)

    # verify that the supplied ArUCo tag exists and is supported by
    # OpenCV
    if ARUCO_DICT.get(aruco_dict, None) is None:
        print("[INFO] ArUCo tag of '{}' is not supported".format(
            aruco_dict))
        sys.exit(0)

    # load the ArUCo dictionary, grab the ArUCo parameters, and detect
    # the markers
    print("[INFO] detecting '{}' tags...".format(aruco_dict))
    arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_dict])
    arucoParams = cv2.aruco.DetectorParameters_create()

    # # write video
    # video_out = cv2.VideoWriter(
    #     'aruco.avi', cv2.VideoWriter_fourcc(*'MJPG'), 20.0, (600, 337))

    while True:
        ret, frame = video_capture.read()
        # exit loop at the end of the video
        if ret == False:
            print("End of video")
            break

        (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict,
                                                           parameters=arucoParams)
        
        # verify at least one ArUco marker was detected
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()

            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                if markerID == 3:
                    # extract the marker corners (which are always returned in
                    # top-left, top-right, bottom-right, and bottom-left order)
                    corners = markerCorner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corners

                    # convert each of the (x, y)-coordinate pairs to integers
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))

                    # draw the bounding box of the ArUCo detection
                    cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
                    cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                    cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                    cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

                    # compute and draw the center (x, y)-coordinates of the ArUco
                    # marker
                    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                    cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

                    # print(frame.shape)
                    # eY = frame.shape[0]/2 - cY
                    eX = frame.shape[1]/2 - cX
                    print("eX: {}".format(eX))
                    # if eX > 0, marker is on the leftside of frame
                    # if eX < 0, marker is on the rightside of frame
                    

                    # draw the ArUco marker ID on the image
                    cv2.putText(frame, str(markerID),
                                (topLeft[0], topLeft[1] -
                                15), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 255, 0), 2)
                # print("[INFO] ArUco marker ID: {}".format(markerID))

        frame = imutils.resize(frame, height=800)
        # Display the resulting frame
        cv2.imshow('frame', frame)
        # video_out.write(frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release everything:
    # video_out.release()
    video_capture.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    aruco_dict = "DICT_4X4_50"
    main(aruco_dict)
