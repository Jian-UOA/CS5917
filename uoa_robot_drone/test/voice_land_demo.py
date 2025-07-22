import time
import threading
import speech_recognition as sr
import pyttsx3

# 语音播报函数
def speak(text):
    engine = pyttsx3.init()

    voices = engine.getProperty('voices')
    # 选择更自然的英文voice（如en_GB或en_US，通常女声更自然）
    for v in voices:
        if 'en' in v.id and ('female' in v.name.lower() or 'zira' in v.id.lower()):
            engine.setProperty('voice', v.id)
            break
    engine.setProperty('rate', 120)  # 设置语速
    engine.say(text)
    engine.runAndWait()

# 语音识别线程
def listen_for_land_command(trigger_phrase, on_land_callback):
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
                if trigger_phrase in result:
                    on_land_callback()
                    break
            except sr.UnknownValueError:
                continue
            except Exception as e:
                print(f"Speech recognition error: {e}")
                continue

def main():
    # 1. 返航悬停后，播放提示
    speak("Aberdeen number one express drone has completed delivery and is hovering, waiting for the land command.")

    # 2. 启动监听线程，等待语音指令
    def on_land():
        speak("Number one express drone is landing now.")
        print("[模拟] 执行降落指令: tello.land()")
        time.sleep(3)
        speak("This delivery service is complete. Have a nice day. Goodbye.")

    listen_for_land_command("number one can land", on_land)

if __name__ == "__main__":
    main()
