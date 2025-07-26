import time
import asyncio
import os
import speech_recognition as sr
import edge_tts

 # Speech synthesis function (edge-tts, offline Microsoft TTS, supports various natural female voices)
async def speak(text, voice="en-GB-LibbyNeural"):
    communicate = edge_tts.Communicate(text, voice=voice)
    await communicate.save("temp_tts.mp3")
    os.system("mpg123 temp_tts.mp3")
    time.sleep(0.5)  # Slight delay after playback to ensure complete audio
    os.remove("temp_tts.mp3")

 # Voice recognition thread
def listen_for_land_command(on_land_callback):
    recognizer = sr.Recognizer()
    mic = sr.Microphone()
    with mic as source:
        recognizer.adjust_for_ambient_noise(source)
        print("Listening for land command...")
        while True:
            audio = recognizer.listen(source, timeout=None)
            try:
                result = recognizer.recognize_google(audio, language='en-GB')
                print(f"Heard: {result}")
                should_exit = on_land_callback(result)
                if should_exit:
                    break
            except sr.UnknownValueError:
                continue
            except Exception as e:
                print(f"Speech recognition error: {e}")
                continue



def main():
    # 1. After returning and hovering, play the prompt
    asyncio.run(speak("Aberdeen number one express drone has completed delivery and is hovering, waiting for the land command."))
    
    # 2. Start listening thread, wait for voice command
    def on_land(command_text):
        if command_text == "number one can land":
            asyncio.run(speak("Number one express drone is landing now."))
            print("[Simulated] Execute land command: tello.land()")
            time.sleep(3)
            asyncio.run(speak("This delivery service is complete. Have a nice day. Goodbye."))
            return True
        return False

    listen_for_land_command(on_land)

if __name__ == "__main__":
    main()
