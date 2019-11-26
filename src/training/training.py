import numpy as np
import cv2
import glob

from keras import layers
from keras import models
from keras import optimizers
from keras.utils import plot_model
from keras import backend

PATH = "/home/kausik/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/training/pictures"
LETTER_WIDTH_THRESH = 60
LETTER_HEIGHT_THRESH = 60
VALIDATION_SPLIT = 0.2
BLUE_MASK = [np.array([100,100,30]), np.array([140,255,255])]

def hotlocalsingles(char):
    index = ord(char)
    index = index - 48 if index < 58 else index - 55
    onehot = [0]*36
    onehot[index] = 1
    return onehot

chars = []
output = []

for filename in glob.glob(PATH + '/*.png'):
    platehsv = cv2.imread(filename)
    platehsv = cv2.cvtColor(platehsv, cv2.COLOR_BGR2HSV)
    platehsv = cv2.inRange(platehsv, *BLUE_MASK)

    _, morecnts, _ = cv2.findContours(platehsv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boundRect = [None]*len(morecnts)
    contours_poly = [None]*len(morecnts)

    for i, c in enumerate(morecnts):
        contours_poly[i] = cv2.approxPolyDP(c, 3, True)
        boundRect[i] = cv2.boundingRect(contours_poly[i])

    boundRect = sorted(boundRect, key=lambda x: x[0])
    
    for i, c in enumerate(boundRect):
        if (300 > c[0] and 300 < c[0] + c[2]) or c[3] < LETTER_HEIGHT_THRESH:
            pass
        elif c[2] > LETTER_WIDTH_THRESH and c[2] < 2 * LETTER_WIDTH_THRESH:
            chars.append(cv2.resize(platehsv[c[1]:c[1]+c[3], c[0]:c[0]+c[2]], (106, 160)))
        elif c[2] >= 2 * LETTER_WIDTH_THRESH:
            chars.append(cv2.resize(platehsv[c[1]:c[1]+c[3], c[0]:c[0]+int(c[2]/2.0)], (106, 160)))
            chars.append(cv2.resize(platehsv[c[1]:c[1]+c[3], c[0]+int(c[2]/2.0):c[0]+c[2]], (106, 160)))
    
    output.extend([hotlocalsingles(i) for i in filename[-8:-4]])

chars = np.array(chars)[:,:,:,np.newaxis]
output = np.array(output)
print(str(chars.shape))
print(str(output.shape))

def reset_weights(model):
    session = backend.get_session()
    for layer in model.layers: 
        if hasattr(layer, 'kernel_initializer'):
            layer.kernel.initializer.run(session=session)
            
conv_model = models.Sequential()
conv_model.add(layers.Conv2D(32, (3, 3), activation='relu',
                             input_shape=(160, 106, 1)))
conv_model.add(layers.MaxPooling2D((2, 2)))
conv_model.add(layers.Conv2D(64, (3, 3), activation='relu'))
conv_model.add(layers.MaxPooling2D((2, 2)))
conv_model.add(layers.Conv2D(128, (3, 3), activation='relu'))
conv_model.add(layers.MaxPooling2D((2, 2)))
conv_model.add(layers.Conv2D(128, (3, 3), activation='relu'))
conv_model.add(layers.MaxPooling2D((2, 2)))
conv_model.add(layers.Flatten())
conv_model.add(layers.Dropout(0.5))
conv_model.add(layers.Dense(512, activation='relu'))
conv_model.add(layers.Dense(36, activation='softmax'))

LEARNING_RATE = 1e-4
conv_model.compile(loss='categorical_crossentropy',
                   optimizer=optimizers.RMSprop(lr=LEARNING_RATE),
                   metrics=['acc'])
reset_weights(conv_model)


history_conv = conv_model.fit(chars, output, 
                              validation_split=VALIDATION_SPLIT, 
                              epochs=20, batch_size=16)

conv_model.save('plate.h5')

# import sklearn
# from sklearn import metrics

# y_pred = conv_model.predict(chars)
# y_pred = [np.argmax(i) for i in y_pred]
# y_data = [np.argmax(i) for i in output]

# confusion_matrix = sklearn.metrics.confusion_matrix(y_data, y_pred)
# confusion_matrix = confusion_matrix.astype("float") / confusion_matrix.sum(axis=1)[:, np.newaxis]
# print(confusion_matrix)

# import matplotlib.pyplot as plt

# plt.plot(history_conv.history['loss'])
# plt.plot(history_conv.history['val_loss'])
# plt.title('model loss')
# plt.ylabel('loss')
# plt.xlabel('epoch')
# plt.legend(['train loss', 'val loss'], loc='upper right')
# plt.show()

# import matplotlib.pyplot as plt

# plt.plot(history_conv.history['acc'])
# plt.plot(history_conv.history['val_acc'])
# plt.title('model accuracy')
# plt.ylabel('accuracy (%)')
# plt.xlabel('epoch')
# plt.legend(['train accuracy', 'val accuracy'], loc='lower right')
# plt.show()