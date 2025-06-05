import os
import wave
import math

def split_wav_into_seconds(input_file, output_dir, segment_length=1):
    """
    Splits a WAV file into 1-second segments and saves them as separate WAV files.

    Args:
        input_file (str): Path to the input WAV file
        output_dir (str): Directory to save the output segments
        segment_length (int): Length of each segment in seconds (default: 1)
    """
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)

    # Open the input WAV file
    with wave.open(input_file, 'rb') as wav_file:
        # Get audio parameters
        params = wav_file.getparams()
        n_channels, sampwidth, framerate, n_frames = params[:4]

        # Calculate total duration and number of segments
        total_duration = n_frames / framerate
        num_segments = math.ceil(total_duration / segment_length)
        frames_per_segment = framerate * segment_length

        print(f"Splitting {input_file} into {num_segments} {segment_length}-second segments")
        print(f"Audio parameters: {n_channels} channels, {sampwidth} bytes/sample, {framerate} Hz")

        # Read all frames at once (for efficiency)
        all_frames = wav_file.readframes(n_frames)

        for i in range(num_segments):
            # Calculate start and end frames
            start = i * frames_per_segment
            end = start + frames_per_segment

            # Get frames for this segment
            segment_frames = all_frames[start*sampwidth*n_channels : end*sampwidth*n_channels]

            # Create output filename
            output_filename = os.path.join(
                output_dir,
                f"{os.path.splitext(os.path.basename(input_file))[0]}_segment_{i+1:04d}.wav"
            )

            # Write the segment to a new WAV file
            with wave.open(output_filename, 'wb') as segment_file:
                segment_file.setparams(params)
                segment_file.writeframes(segment_frames)

            print(f"Saved segment {i+1}/{num_segments} to {output_filename}")

    print(f"\nFinished splitting. All segments saved to {output_dir}")

if __name__ == "__main__":
    # Example usage
    input_wav = "recording.wav"  # Replace with your input file
    output_directory = "./samples/6/"  # Directory for output files

    split_wav_into_seconds(input_wav, output_directory)
