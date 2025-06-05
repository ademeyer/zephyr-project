import os
os.environ['CUDA_VISIBLE_DEVICES'] = '-1'  # Force TensorFlow to use CPU only

import tensorflow as tf
from tensorflow.keras import layers, models # type: ignore
from tensorflow.keras.models import load_model #type: ignore
import numpy as np
import librosa
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split

# Configure TensorFlow for CPU optimization
tf.config.threading.set_intra_op_parallelism_threads(4)
tf.config.threading.set_inter_op_parallelism_threads(4)


print("TensorFlow version:", tf.__version__)
print("Using CPU:", len(tf.config.list_physical_devices('GPU')) == 0)

# Parameter
sample_rate = 16000 # standard for speech recognition
duration = 1        # Duration in seconds of each audio clip
hop_length = 512    # for STFT
n_mels = 64         # Number of Mel bands
n_fft = 2048        # FFT window size


def audio_to_melspectrogram(audio_path):
    # Load audio file from path
    y, sr = librosa.load(audio_path, sr=sample_rate, duration=duration)

    # pad or trim audio to desired duration
    if len(y) < sample_rate * duration:
        y = np.pad(y, (0, max(0, sample_rate * duration - len(y))), mode='constant')
    else:
        y = y[: sample_rate * duration]

    # Convert to mel spectrogram
    S = librosa.feature.melspectrogram( y=y, sr=sr, n_mels=n_mels,
                                        n_fft=n_fft, hop_length=hop_length)

    S_db = librosa.power_to_db(S, ref=np.max)

    # Normalize
    S_db = (S_db - S_db.min()) / (S_db.max() - S_db.min())

    # Add channel dimension (CNN expects 3D input)
    S_db = np.expand_dims(S_db, axis=1)

    return S_db

# Usage
# spectrogram = audio_to_melspectrogram('./samples/cheer.wav')

def load_audio_dataset(data_dir, max_files_per_class=500):
    """Load and preprocess audio dataset with CPU optimizations"""
    X = []
    y = []
    class_names = sorted(os.listdir(data_dir))

    for label, class_name in enumerate(class_names):
        class_dir = os.path.join(data_dir, class_name)
        audio_files = [f for f in os.listdir(class_dir) if f.endswith('.wav')][:max_files_per_class]

        for audio_file in audio_files:
            audio_path = os.path.join(class_dir, audio_file)
            spectrogram = audio_to_melspectrogram(audio_path)
            if spectrogram is not None:
                X.append(spectrogram)
                y.append(label)

    X = np.array(X)
    y = tf.keras.utils.to_categorical(y, num_classes=len(class_names))

    return X, y, class_names

# Load dataset
data_dir = './samples/' # 'path/to/your/dataset'  # Should contain subfolder for each class
X, y, class_names = load_audio_dataset(data_dir)

# '''
# Split dataset (stratified for class balance)
X_train, X_temp, y_train, y_temp = train_test_split(
    X, y, test_size=0.3, random_state=42, stratify=y)
X_val, X_test, y_val, y_test = train_test_split(
    X_temp, y_temp, test_size=0.5, random_state=42, stratify=y_temp)

# 1. Reshape your data
X_train = np.transpose(X_train, (0, 1, 3, 2))  # From (N,64,1,32) to (N,64,32,1)
X_val = np.transpose(X_val, (0, 1, 3, 2))
X_test = np.transpose(X_test, (0, 1, 3, 2))

# Save X_train to disk
np.save('X_train_audio.npy', X_train)

print(f"Training samples: {len(X_train)}")
print(f"Validation samples: {len(X_val)}")
print(f"Test samples: {len(X_test)}")

def create_audio_cnn(input_shape, num_classes):
    """Create a CPU-friendly CNN model for audio classification"""
    model = models.Sequential([
        layers.Input(shape=input_shape),

        # Reduced conv blocks
        layers.Conv2D(16, (3,3), activation='relu', padding='same'),
        layers.BatchNormalization(),
        layers.MaxPooling2D((2,2)),
        layers.Dropout(0.25),

        layers.Conv2D(32, (3,3), activation='relu', padding='same'),
        layers.BatchNormalization(),
        layers.MaxPooling2D((2,2)),
        layers.Dropout(0.25),

        layers.GlobalAveragePooling2D(),
        layers.Dense(32, activation='relu'),
        layers.Dropout(0.5),
        layers.Dense(num_classes, activation='softmax')
    ])
    return model

# Calculate input shape
time_steps = (sample_rate * duration) // hop_length + 1  # For 1 second audio with hop_length=512
input_shape = (n_mels, time_steps, 1)

model = create_audio_cnn(input_shape, len(class_names))

# Compile with CPU-friendly settings
model.compile(optimizer='adam',
              loss='categorical_crossentropy',
              metrics=['accuracy'])

model.summary()

# Callbacks for CPU training
callbacks = [
    tf.keras.callbacks.EarlyStopping(
        patience=8,
        restore_best_weights=True,
        monitor='val_accuracy'),
    tf.keras.callbacks.ModelCheckpoint(
        'best_audio_model_cpu.h5',
        save_best_only=True,
        monitor='val_accuracy'),
    tf.keras.callbacks.ReduceLROnPlateau(
        factor=0.5,
        patience=3,
        min_lr=1e-6)
]

# Train with smaller batch size for CPU memory
history = model.fit(
    X_train, y_train,
    batch_size=16,  # Reduced for CPU
    epochs=50,
    validation_data=(X_val, y_val),
    callbacks=callbacks,
    verbose=1)

# Evaluate on test set
test_loss, test_acc = model.evaluate(X_test, y_test)
print(f'Test accuracy: {test_acc:.4f}')


# Plot training history
def plot_training_history(history):
    plt.figure(figsize=(12, 4))

    plt.subplot(1, 2, 1)
    plt.plot(history.history['accuracy'], label='Train Accuracy')
    plt.plot(history.history['val_accuracy'], label='Validation Accuracy')
    plt.title('Model Accuracy')
    plt.ylabel('Accuracy')
    plt.xlabel('Epoch')
    plt.legend()

    plt.subplot(1, 2, 2)
    plt.plot(history.history['loss'], label='Train Loss')
    plt.plot(history.history['val_loss'], label='Validation Loss')
    plt.title('Model Loss')
    plt.ylabel('Loss')
    plt.xlabel('Epoch')
    plt.legend()

    plt.tight_layout()
    plt.show()

plot_training_history(history)

# Save the final model
model.save('audio_classification_cpu.keras')

print("Training data shape:", X_train.shape)  # Should be (N, 64, 32, 1)
print("Model input shape:", model.input_shape)  # Should be (None, 64, 32, 1)

# '''

def predict_audio_class(audio_path, model, class_names):
    """Predict class for a single audio file"""
    spectrogram = audio_to_melspectrogram(audio_path)
    if spectrogram is None:
        return None

    spectrogram = np.expand_dims(spectrogram, axis=0)  # Add batch dimension
    spectrogram = np.transpose(spectrogram, (0, 1, 3, 2))
    predictions = model.predict(spectrogram, verbose=0)
    predicted_class = np.argmax(predictions[0])
    confidence = np.max(predictions[0])

    return class_names[predicted_class], confidence

# Example usage
myMode = load_model('./audio_classification_cpu.keras')

predicted_class, confidence = predict_audio_class('./samples/2/recording2.wav', myMode, class_names) #model, class_names)
print(f"Predicted: {predicted_class} (Confidence: {confidence:.2f})")
