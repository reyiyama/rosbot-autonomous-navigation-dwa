import cv2
import os

def generate_single_marker(aruco_dict):
    marker_size = int(input("Enter the marker size: "))
    marker_id = int(input("Enter the marker ID: "))
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

    # Specify the path to save the marker
    save_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'resource')
    if not os.path.exists(save_path):
        os.makedirs(save_path)

    file_path = os.path.join(save_path, "marker_{}.png".format(marker_id))
    cv2.imwrite(file_path, marker_img)

    marker_img = cv2.imread(file_path)
    cv2.imshow("Marker", marker_img)
    print("Dimensions:", marker_img.shape)
    cv2.waitKey(0)

def generate_bulk_markers(aruco_dict):
    marker_size = int(input("Enter the marker size: "))
    num_markers = int(input("Enter the number of markers to generate: "))
    marker_imgs = []

    # Specify the path to save the markers
    save_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'resource')
    if not os.path.exists(save_path):
        os.makedirs(save_path)

    for marker_id in range(num_markers):
        marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
        file_path = os.path.join(save_path, "marker_{}.png".format(marker_id))
        cv2.imwrite(file_path, marker_img)
        marker_imgs.append(cv2.imread(file_path))

    for marker_img in marker_imgs:
        cv2.imshow("Marker", marker_img)
        print("Dimensions:", marker_img.shape)
        cv2.waitKey(0)

def main():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    user_input = input("Press '1' to generate a single marker or "
                       "'2' to generate markers in bulk: ")

    if user_input == "1":
        generate_single_marker(aruco_dict)
    elif user_input == "2":
        generate_bulk_markers(aruco_dict)
    else:
        print("Invalid input. Please try again.")

if __name__ == "__main__":
    main()