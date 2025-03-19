# Whisper Server 

## Overview
This project implements a server to receive and process the audio from the client(raspberry pi).The whisper model used in this project is for faster whisper only.

---

## Requirements

* Python 3.9 or greater

Unlike openai-whisper, FFmpeg does **not** need to be installed on the system. The audio is decoded with the Python library [PyAV](https://github.com/PyAV-Org/PyAV) which bundles the FFmpeg libraries in its package.

---

## Installation

The module can be installed from [PyPI](https://pypi.org/project/faster-whisper/):

```bash
pip install faster-whisper
```

### 1. Step to run the server

clone the faster whisper repository from guthub

```bash
git clone https://github.com/SYSTRAN/faster-whisper.git
```

---

### 2. move the laptop_server.py to faster whisper repo

To move `laptop_server.py` to the `tests/` folder in the Faster Whisper repository, follow these steps:

1. Navigate to the Faster Whisper repository:
   ```bash
   cd /path/to/faster-whisper
   ```
2. move the `laptop_server.py` to `tests/` folder
   ```bash
   mv laptop_server.py tests/
   ```
 3. Verify that the file has been moved:
   ```bash
   ls tests/
   ```
---

### 3. Run the laptop_server.py
Example Usage

```sh
python3 tests/laptop_server.py --host 0.0.0.0 --port 12345 --model-path /home/hn/Downloads/drone_model --device cpu --compute int8
```
the port number must be same as the client port nnumber.
The model path can remove (to use default whisper model).
If using local model the model path should be your local model directory
The device can select `cpu` or `cuda` .

---

## Model conversion

[downlaod fine tuned model from Hugging Face Hub](https://huggingface.co/CSY1109).
select suitable fine tuned model.

### 1. convert hugging face model to Ctranslate2 format
example usage

```bash
python convert_model.py --model_id CSY1109/drone_sy_tiny_t3 --force
```
* The option `--model` accepts a model name on the Hub or a path to a model directory.
* If the option `--copy_files tokenizer.json` is not used, the tokenizer configuration is automatically downloaded when the model is loaded later.

Models can also be converted from the code. See the [conversion API](https://opennmt.net/CTranslate2/python/ctranslate2.converters.TransformersConverter.html).

---

## License
This project is licensed under the **MIT License**.

## Author
Lim Kai Shan, Chang Siang Yi, Teh Jin Qian
