import os
import tensorflow as tf
assert tf.__version__.startswith('2')

from mediapipe_model_maker import image_classifier

import matplotlib.pyplot as plt

print("Num GPUs Available: ", len(tf.config.list_physical_devices('GPU')))
print("tensorflow version", tf.__version__)


image_path = "./dataset"


print(image_path)
labels = []
for i in os.listdir(image_path):
  if os.path.isdir(os.path.join(image_path, i)):
    labels.append(i)
print(labels)


data = image_classifier.Dataset.from_folder(image_path)
train_data, validation_data = data.split(0.7)


spec = image_classifier.SupportedModels.EFFICIENTNET_LITE4
hparams = image_classifier.HParams(export_dir="exported_model", epochs=100, batch_size=10)
options = image_classifier.ImageClassifierOptions(supported_model=spec, hparams=hparams)


model = image_classifier.ImageClassifier.create(
    train_data = train_data,
    validation_data = validation_data,
    options=options,
)


model.export_model()
