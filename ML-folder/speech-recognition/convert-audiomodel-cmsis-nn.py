import tensorflow as tf
import numpy as np

# Load trained model
model = tf.keras.models.load_model('./best_audio_model_cpu.h5')

# Representative dataset for quantization calibration
def representative_dataset():
    # Load your saved data
    X_train = np.load('X_train_audio.npy')

   # Calculate mean and std for normalization
    mean = np.mean(X_train)
    std = np.std(X_train)

    # Use first 200 samples (adjust as needed)
    for i in range(min(200, len(X_train))):
        # Normalize the sample
        sample = (X_train[i:i+1] - mean) / std
        yield [sample.astype(np.float32)]

converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.representative_dataset = representative_dataset
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
converter.inference_input_type = tf.int8  # Or tf.uint8
converter.inference_output_type = tf.int8  # Or tf.uint8

# 4. Perform the conversion
try:
    tflite_model = converter.convert()

    # Save the quantized model
    with open('quantized_audio_model.tflite', 'wb') as f:
        f.write(tflite_model)
    print("Model successfully converted and saved!")

except Exception as e:
    print(f"Conversion failed: {str(e)}")
    exit()

# 5. Verify the quantized model
interpreter = tf.lite.Interpreter(model_content=tflite_model)
interpreter.allocate_tensors()

# Get input/output details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Print quantization parameters
print("\nQuantization parameters:")
print(f"Input scale: {input_details[0]['quantization'][0]}")
print(f"Input zero point: {input_details[0]['quantization'][1]}")

# 6. Prepare sample input (properly quantized)
sample_input = np.load('X_train_audio.npy')[0:1]  # Get first sample
mean = np.mean(sample_input)
std = np.std(sample_input)

# Normalize and quantize
input_scale, input_zero_point = input_details[0]['quantization']
quantized_input = (sample_input - mean) / std  # Normalize
quantized_input = quantized_input / input_scale + input_zero_point  # Quantize
quantized_input = np.round(quantized_input).astype(np.int8)  # Convert to int8

# 7. Run inference
try:
    interpreter.set_tensor(input_details[0]['index'], quantized_input)
    interpreter.invoke()
    output = interpreter.get_tensor(output_details[0]['index'])

    # De-quantize output if needed
    output_scale, output_zero_point = output_details[0]['quantization']
    if output_scale:
        output = (output.astype(np.float32) - output_zero_point) * output_scale

    print("\nInference successful!")
    print("Output:", output)

except Exception as e:
    print(f"Inference failed: {str(e)}")

# Convert to C array format
def convert_to_c_array(bytes_model):
    hex_array = []
    for i, b in enumerate(bytes_model):
        if i % 12 == 0:
            hex_array.append('\n')
        hex_array.append(f'0x{b:02x}, ')
    return ''.join(hex_array)

model_size_kb = len(tflite_model) / 1024
print(f"Model size: {model_size_kb:.2f} KB")

# Generate C header file
with open('model.h', 'w') as f:
    f.write('// TensorFlow Lite model for CMSIS-NN\n')
    f.write('#ifndef MODEL_H\n')
    f.write('#define MODEL_H\n\n')
    f.write(f'const unsigned char model_data[] = {{{convert_to_c_array(tflite_model)}}};\n')
    f.write(f'const unsigned int model_len = {len(tflite_model)};\n\n')
    f.write('#endif // MODEL_H\n')

print('Embedded model created successfully')
