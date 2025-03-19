#!/usr/bin/env python3
import os
import torch
import shutil
import argparse
from pathlib import Path
from huggingface_hub import snapshot_download
import faster_whisper

def convert_hf_to_faster_whisper(hf_model_id, output_dir=None, force=False):
    """
    Convert a Hugging Face Whisper model to faster-whisper format
    
    Args:
        hf_model_id (str): Hugging Face model ID (e.g., "openai/whisper-large-v3")
        output_dir (str, optional): Directory to save the converted model.
                                   If None, uses model_id name in current directory.
        force (bool): Whether to overwrite the output directory if it exists.
    
    Returns:
        str: Path to the converted model directory
    """
    print(f"Converting {hf_model_id} to faster-whisper format...")
    
    # Create output directory if not specified
    if output_dir is None:
        model_name = hf_model_id.split("/")[-1]
        output_dir = f"{model_name}-ct2"
    
    print(f"Output directory: {output_dir}")
    print(f"Force overwrite: {force}")
    
    # Download the Hugging Face model
    print(f"Downloading {hf_model_id} from Hugging Face...")
    model_dir = snapshot_download(repo_id=hf_model_id)
    
    # Import CTranslate2 for conversion
    try:
        import ctranslate2
    except ImportError:
        raise ImportError(
            "CTranslate2 is required for this conversion. "
            "Please install it with: pip install ctranslate2"
        )
    
    # Get the converter
    converter = ctranslate2.converters.TransformersConverter(model_dir)
    
    # Inspect the available parameters for the convert method
    import inspect
    converter_params = inspect.signature(converter.convert).parameters
    print(f"Available converter parameters: {list(converter_params.keys())}")
    
    # Convert the model with force parameter if available
    print("Converting model...")
    if "force" in converter_params:
        converter.convert(output_dir, quantization="float16", force=force)
    else:
        # If force parameter is not available, manually handle it
        if os.path.exists(output_dir) and force:
            print(f"Manually removing existing output directory: {output_dir}")
            shutil.rmtree(output_dir)
        converter.convert(output_dir, quantization="float16")
    
    # Copy necessary files manually
    print("Copying configuration files...")
    files_to_copy = ["tokenizer.json", "config.json", "generation_config.json", "preprocessor_config.json"]
    for filename in files_to_copy:
        src_path = os.path.join(model_dir, filename)
        if os.path.exists(src_path):
            shutil.copy(src_path, os.path.join(output_dir, filename))
            print(f"Copied {filename}")
        else:
            print(f"Warning: {filename} not found in source model")
    
    print(f"Model successfully converted and saved to: {output_dir}")
    print(f"You can now use it with: model = faster_whisper.WhisperModel('{output_dir}')")
    
    return output_dir

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert Hugging Face Whisper model to faster-whisper format")
    parser.add_argument("--model_id", type=str, default="CSY1109/drone_sy_tiny_t3", 
                        help="Hugging Face model ID")
    parser.add_argument("--output_dir", type=str, default="/home/ubuntu/Downloads/drone_ks_tiny",
                        help="Output directory for the converted model")
    parser.add_argument("--force", action="store_true", 
                        help="Force overwrite of existing output directory")
    
    args = parser.parse_args()
    
    try:
        # Use the parsed arguments
        converted_model_path = convert_hf_to_faster_whisper(
            hf_model_id=args.model_id, 
            output_dir=args.output_dir, 
            force=args.force
        )
        
        # Test the converted model
        model = faster_whisper.WhisperModel(converted_model_path)
        print("Model loaded successfully!")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

        # python convert_hf_to_faster_whisper.py --model_id CSY1109/drone_sy_tiny_t3 --force
