import click
from datetime import datetime
import io
import os
import soundfile as sf
from tempfile import NamedTemporaryFile
import time
import torch
import torchaudio
import traceback
from transformers import AutoProcessor, SeamlessM4Tv2Model
from typing import Optional

FOLDER_NAME = 'data'

def waveform_to_audio(waveform: torch.Tensor) -> bytes:
    waveform = waveform.squeeze().cpu().numpy()
    with NamedTemporaryFile(suffix=".wav", delete=False) as f:
        sf.write(f.name, waveform, 22050)
        f.seek(0)
        audio = f.read()
    return audio


def audio_to_waveform(audio: bytes) -> (torch.Tensor, int):
    with NamedTemporaryFile(suffix=".wav", mode='wb+', delete=False) as f:
        f.write(audio)
        f.seek(0)
        waveform, sample_rate = torchaudio.load(f.name)
    return waveform, sample_rate


class SeamlessMT:
    def __init__(
            self,
            model_name="facebook/seamless-m4t-v2-large",
            save_to_wav_file:bool=False,
            device: Optional[torch.device] = None
        ):

        self.processor = None
        self.model = None
        self.model_name = model_name
        self.device = device or torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print("Found device : ", self.device)

        if save_to_wav_file:
            self.save_to_wav_file = save_to_wav_file
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")  # Format: YYYYMMDD_HHMMSS
            filename = "output_" + timestamp + ".wav"
            self.filepath = os.path.join(FOLDER_NAME, filename)
            if not os.path.exists(os.path.dirname(self.filepath)):
                os.makedirs(os.path.dirname(self.filepath))
        else:
            self.save_to_wav_file = False

    def init(self):
        self.processor = AutoProcessor.from_pretrained(self.model_name)
        self.model = SeamlessM4Tv2Model.from_pretrained(self.model_name).to(self.device)
        self.model.eval()

    def tts(self, text: str, source_language: str = 'eng', target_language: str = 'eng') -> bytes:
        if self.processor is None or self.model is None:
            self.init()
        with torch.no_grad():
            text_inputs = self.processor(text=text,
                                        src_lang=source_language,
                                        return_tensors="pt")
            try:
                audio_array_from_text = self.model.generate(**text_inputs, tgt_lang=target_language)[
                    0].cpu().numpy().squeeze()
            except Exception as e:
                print(f"Error generating audio from text: {e}")
                traceback.print_exc()
                return None
        audio = io.BytesIO()

        sf.write(audio, audio_array_from_text, samplerate=16000, format='wav')
        audio.seek(0)
        audio_bytes = audio.read()
        return audio_bytes

    def stt(self, audio: bytes, source_language='eng', target_language='eng') -> str:
        if self.processor is None or self.model is None:
            self.init()
        waveform, sample_rate = audio_to_waveform(audio)

        if self.save_to_wav_file:
            torchaudio.save(self.filepath, waveform, sample_rate)

        # Move the waveform to device
        waveform = waveform

        audio = torchaudio.functional.resample(waveform, sample_rate, 16000)
        
        with torch.no_grad():
            try:
                audio_inputs = self.processor(audios=audio, return_tensors="pt").to(self.device)
            except Exception as e:
                print(f"Error processing audio: {e}")
                traceback.print_exc()
                return None

            try:
                text_output = self.model.generate(**audio_inputs, tgt_lang=target_language, generate_speech=False)
            except Exception as e:
                print(f"Error generating text from audio: {e}")
                traceback.print_exc()
                print("Device : ", self.model.device)
                return None

            recognized_text = self.processor.decode(text_output[0].tolist()[0], skip_special_tokens=True)
        return recognized_text



@click.command()
@click.option("--benchmark", type=bool, default=False, is_flag=True)
def main(benchmark:bool):

    if benchmark:
        parent_dir = os.path.abspath(os.path.join(os.getcwd(), ".."))
        data_folder = os.path.join(parent_dir, FOLDER_NAME)
        wav_benchmark_files = [f for f in os.listdir(data_folder) if f.startswith("benchmark_") and f.endswith(".wav")]

        devices = [torch.device('cuda') , torch.device('cpu')]
        iterations = 10

        for device in devices:
            seamless = SeamlessMT(device=device)
            seamless.init()
            for wav_file in wav_benchmark_files:
                file_path = os.path.abspath(os.path.join(data_folder, wav_file))
                print(f"Processing file : {file_path}")
                waveform, _ = torchaudio.load(file_path)
                audio = waveform_to_audio(waveform=waveform)

                execution_time_list = []
                for i in range(iterations):
                    start_time = time.time()
                    text = seamless.stt(audio)
                    execution_time = time.time() - start_time
                    print(f"Transcribed text = {text}")

                    execution_time_list.append(execution_time)

                print(f"Execution times for {wav_file} over {iterations} iterations on {device}")
                print(f"{execution_time_list=} \nAvg execution time {sum(execution_time_list)/len(execution_time_list)}\n\n")
            
    else:
        seamless = SeamlessMT()
        seamless.init()
        while True:
            text_input = input("Enter text to translate: ")
            if text_input == "exit":
                break
            audio = seamless.tts(text_input)
            # print(f"Audio: {audio[:10]}...")

            text_output = seamless.stt(audio)
            print(f"Text: {text_output}")
    print("Exiting...")

if __name__ == "__main__":
    main()
