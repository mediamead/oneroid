
import numpy as np
import tensorflow as tf
from matplotlib import pyplot as plt
from IPython.display import clear_output

# plots loss and metrics for training and validation
def plot_fh(history, plot_val_loss=True):
  plt.plot(history.history['loss'])
  if plot_val_loss:
    plt.plot(history.history['val_loss'])
  plt.title('model loss')
  plt.ylabel('loss')
  plt.xlabel('epoch')
  if plot_val_loss:
    plt.legend(['train', 'val'], loc='upper left')
  else:
    plt.legend(['train'], loc='upper left')
  plt.show()

# plots given data_dict
def live_plot(data_dict, figsize=(17,15), title=''):
    clear_output(wait=True)
    plt.figure(figsize=figsize)
    for label,data in data_dict.items():
        plt.plot(data, label=label)
    plt.title(title)
    plt.grid(True)
    plt.xlabel('epoch')
    plt.legend(loc='center left') # the plot evolves to the right
    plt.show()

def mk_model(N=128, L=4, activation="relu"):
    inputs = layer = tf.keras.Input(shape=(7,))
    
    for _ in range(L):
        layer = tf.keras.layers.Dense(N, activation=activation)(layer)
        
    outputs = tf.keras.layers.Dense(6, activation='tanh')(layer)

    model = tf.keras.Model(inputs, outputs)
    model.compile(loss='mean_squared_error', optimizer='adam', metrics=['mae'])

    return model
