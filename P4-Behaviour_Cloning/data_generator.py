from pathlib import Path, PureWindowsPath

import cv2
import numpy as np
from sklearn.utils import shuffle


def data_generator(records, batch_size, current_image_path):
    """

    :param list records: List of records loaded from csv file
    :param int batch_size: Batch size.
    :param str current_image_path: Image path
    :return:
    """

    while 1:
        shuffle(records)

        for index in range(0, len(records), batch_size):
            batch_records = records[index:index + batch_size]

            images = []
            angles = []

            for batch_record in batch_records:
                center_path = Path.joinpath(Path(current_image_path), PureWindowsPath(batch_record[0]).parts[-1])
                center_image = cv2.imread(str(center_path))

                left_path = Path.joinpath(Path(current_image_path),PureWindowsPath(batch_record[1]).parts[-1])
                left_image = cv2.imread(str(left_path))

                right_path = Path.joinpath(Path(current_image_path),PureWindowsPath(batch_record[2]).parts[-1])
                right_image = cv2.imread(str(right_path))

                # Loading and correcting steering angle for Left and Right perspectives
                correction = 0.2
                center_angle = float(batch_record[3])
                left_angle = center_angle + correction
                right_angle = center_angle - correction

                # Put all the images together
                images.extend([center_image, left_image, right_image])
                angles.extend([center_angle, left_angle, right_angle])

            # Augment images by flipping them.
            augmented_images, augmented_angles = [], []
            for image, angle in zip(images, angles):
                augmented_images.append(image)
                augmented_angles.append(angle)
                augmented_images.append(cv2.flip(image, 1))
                augmented_angles.append(angle * -1.0)

            X_train = np.array(augmented_images)
            y_train = np.array(augmented_angles)

            yield shuffle(X_train, y_train)
