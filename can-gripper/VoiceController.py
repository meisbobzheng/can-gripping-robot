import speech_recognition as sr

class VoiceController:
    def __init__(self, model=None):
        self.model = model  # if you need it for other purposes
        self.recognizer = sr.Recognizer()
        self.mic = sr.Microphone()

    def listen_until_keyword(self, keyword):
        """
        Listens continuously and stops when a keyword is detected.
        :param keywords: list of keywords to match against (lowercase strings)
        """
        print("Listening for keyword:", keyword)

        with self.mic as source:
            self.recognizer.adjust_for_ambient_noise(source)

            while True:
                print("Say something...")
                audio = self.recognizer.listen(source)
                
                try:
                    result = self.recognizer.recognize_sphinx(audio, keyword_entries=[(keyword, 1.0)])
                    print(f"Detected keyword: {result}")
                    return result
                except sr.UnknownValueError:
                    print("Didn't catch that.")
                    continue
                except sr.RequestError as e:
                    print(f"PocketSphinx error: {e}")
                    return None

    def listen_for_initial_command(self, trigger, keywords) -> str | None:
        """
        Listens for an initial command that triggers the main listening loop.
        Resets to trigger if 'cancel' or 'exit' is detected.
        
        :param trigger: the initial command to listen for
        :param keywords: list of keywords to match against (lowercase strings)
        :return: the matched keyword, or None if an error occurs
        """
        exit_keywords = ["cancel", "exit"]
        all_keywords = keywords + exit_keywords

        print("Listening for initial command...")

        with self.mic as source:
            self.recognizer.adjust_for_ambient_noise(source)

            while True:
                # Listen for the trigger word
                while True:
                    print(f"Listening for trigger word: {trigger}")
                    audio = self.recognizer.listen(source)

                    try:
                        result = self.recognizer.recognize_sphinx(audio, keyword_entries=[(trigger, 1.0)])
                        print(f"Trigger word detected: {result}")
                        break  # enter keyword listening loop
                    except sr.UnknownValueError:
                        print("Didn't catch that.")
                        continue
                    except sr.RequestError as e:
                        print(f"PocketSphinx error: {e}")
                        return None

                # Now listen for keywords
                while True:
                    print("Listening for keywords...")
                    audio = self.recognizer.listen(source)

                    try:
                        result = self.recognizer.recognize_sphinx(audio, keyword_entries=[(kw, 1.0) for kw in all_keywords])
                        print(f"Detected keyword: {result}")

                        if result.lower() in exit_keywords:
                            print(f"Resetting due to keyword: {result}")
                            break  # go back to listening for trigger

                        return result  # matched actual keyword
                    except sr.UnknownValueError:
                        print("Didn't catch that.")
                        continue
                    except sr.RequestError as e:
                        print(f"PocketSphinx error: {e}")
                        return None

                


if __name__ == "__main__":
    vc = VoiceController()
    keyword = vc.listen_for_initial_command("hey robot", ["coca-cola", "sprite", "pepsi", "root-beer"])
    print(f"Matched keyword: {keyword}")
