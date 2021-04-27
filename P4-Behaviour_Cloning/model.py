import csv

import matplotlib.pyplot as plt
from keras.layers import Flatten, Dense, Lambda, Cropping2D
from keras.layers.convolutional import Convolution2D
from keras.models import Sequential
from sklearn.model_selection import train_test_split

from data_generator import data_generator


def get_model():
    """

    :return:
    """
    # Model Architecture
    model = Sequential()
    model.add(Lambda(lambda x: x / 127.5 - 1., input_shape=(160, 320, 3)))
    model.add(Cropping2D(cropping=((60, 20), (0, 0))))
    model.add(Convolution2D(24, 5, 5, subsample=(2, 2), activation="relu"))
    model.add(Convolution2D(36, 5, 5, subsample=(2, 2), activation="relu"))
    model.add(Convolution2D(48, 5, 5, subsample=(2, 2), activation="relu"))
    model.add(Convolution2D(64, 3, 3, activation="relu"))
    model.add(Convolution2D(64, 3, 3, activation="relu"))
    model.add(Flatten())
    model.add(Dense(100))
    model.add(Dense(50))
    model.add(Dense(10))
    model.add(Dense(1))

    return model


# Read all the records form the csv file.
csv_records = []
with open('/home/workspace/CarND-Behavioral-Cloning-P3/sim_data3/driving_log.csv') as csv_file:
    reader = csv.reader(csv_file)
    for line in reader:
        csv_records.append(line)

print('CSV File Loaded.')
print("Total number of Records is {}".format(len(csv_records)))

# Define the current location of the images (In case the data was gathered on a different machine)
current_image_path = "/home/workspace/CarND-Behavioral-Cloning-P3/sim_data3/IMG"

# Split the loaded data in to Train and Validation sets.
training_data_samples, validation_data_samples = train_test_split(csv_records, test_size=0.2)

# Get generator objects for the fit_generator method for Training.
train_generator = data_generator(training_data_samples, batch_size=32, current_image_path=current_image_path)
validation_generator = data_generator(validation_data_samples, batch_size=32, current_image_path=current_image_path)

# Model Training
print('Training Started.')

model = get_model()
model.compile(loss='mse', optimizer='adam')
history_object = model.fit_generator(train_generator,
                                     samples_per_epoch=int(len(training_data_samples * 6) / 32),
                                     validation_data=validation_generator,
                                     nb_val_samples=len(validation_data_samples),
                                     nb_epoch=3,
                                     verbose=1)

# Save the model.
model.save('model.h5')
print("Model Saved.")

# Plot training and validation loss.
print(history_object.history.keys())
plt.plot(history_object.history['loss'])
plt.plot(history_object.history['val_loss'])
plt.title('Training/Validation Losses')
plt.ylabel('MSE')
plt.xlabel('Epoch')
plt.legend(['training set', 'validation set'], loc='upper right')
plt.savefig('loss.png')
