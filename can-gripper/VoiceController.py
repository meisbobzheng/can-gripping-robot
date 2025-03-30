import speech_recognition as sr

class VoiceController:
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.mic = sr.Microphone()
        self.exit_keywords = ["cancel", "exit"]

    def listen_for_initial_command(self, trigger: str, keywords: list[str]) -> str | None:
        """
        Listens for an initial trigger phrase (e.g., 'hey robot'), then a follow-up sentence
        that contains one of the target keywords. If 'cancel' or 'exit' is heard, it resets.

        :param trigger: trigger phrase to start the keyword listener
        :param keywords: list of command keywords to match in follow-up
        :return: matched keyword if found, or None if canceled or error
        """
        all_keywords = [kw.lower() for kw in keywords + self.exit_keywords]

        with self.mic as source:
            self.recognizer.adjust_for_ambient_noise(source)

            while True:
                # Step 1: Listen for the trigger
                print(f"Listening for trigger phrase: '{trigger}'...")
                while True:
                    audio = self.recognizer.listen(source)
                    try:
                        result = self.recognizer.recognize_google(audio)
                        print(f"Heard: {result}")

                        if trigger.lower() in result.lower():
                            print("Trigger detected.")
                            break
                    except sr.UnknownValueError:
                        print("Didn't catch that.")
                        continue
                    except sr.RequestError as e:
                        print(f"Google error: {e}")
                        return None

                # Step 2: Listen for follow-up sentence
                print("Listening for follow-up command...")
                while True:
                    audio = self.recognizer.listen(source)
                    try:
                        result = self.recognizer.recognize_google(audio)
                        print(f"Heard follow-up: {result}")
                        spoken = result.lower()

                        for kw in all_keywords:
                            if kw in spoken:
                                if kw in self.exit_keywords:
                                    print(f"Keyword '{kw}' triggered reset.")
                                    break  # go back to listening for trigger
                                print(f"Matched keyword: {kw}")
                                return kw
                    except sr.UnknownValueError:
                        print("Didn't catch that.")
                        continue
                    except sr.RequestError as e:
                        print(f"Google error: {e}")
                        return None
