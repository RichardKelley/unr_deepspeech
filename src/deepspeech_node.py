from deepspeech import Model
import sys
import wave
import struct

class DeepspeechNode:
    # Two ways to pass in model:
    #     model_path: path the pre-trained model
    #     model: model itself
    #
    # dictionary: output from deepspeech is corrected to the closest word
    #             in the given dictionary on a word-by-word basis
    # possibilities: complete output from deepspeech is corrected to the
    #                closest phrase in the given possibilities
    def __init__(self, model=None, model_path=None,
                 possibilities=None, dictionary=None):
        self.model = model
        if model_path != None:
            self.load_model(model_path)
        self.possibilities = possibilities
        self.dictionary = dictionary

    # Default values for n_feaures, n_context, beam_width are from
    # github.com/mozilla/DeepSpeech/blob/master/native_client/python/client.py
    def load_model(self, model_path, n_features=26,n_context=9,beam_width=500):
        model_path += "/"
        alphabet = model_path + "alphabet.txt"
        output_graph = model_path + "output_graph.pb"
        self.model = Model(output_graph, n_features, n_context,
                           alphabet, beam_width)

    # See: en.wikipedia.org/wiki/Levenshtein_distance
    def levenshtein_distance(self, str1, str2):
        if len(str2) > len(str1):
            str2, str1 = str1, str2
        row_count = len(str1)
        col_count = len(str2)

        row_current = range(row_count + 1)
        for row_n in xrange(row_count):
            row_next = [row_n + 1]
            for col_n in xrange(col_count):
                delete_cost = row_current[col_n + 1] + 1
                insert_cost = row_next[col_n] + 1
                substitution_cost = row_current[col_n] + 1
                if str1[row_n] == str2[col_n]:
                    substitution_cost = row_current[col_n]

                row_next.append(min([substitution_cost, delete_cost,
                                     insert_cost]))
            row_current = row_next
        return row_current[-1]

    def stt(self, fs, audio):
        assert self.model != None, "a model must be loaded before testing"
        transcription = self.model.stt(audio, fs)

        if self.dictionary != None:
            transcription_words = transcription.split(" ")

            new_transcription = ""
            for transcribed_word in transcription_words:
                distances=[self.levenshtein_distance(transcribed_word, dict_word)
                           for dict_word in self.dictionary]
                min_dist_index = min(xrange(len(distances)),
                                     key=lambda x: distances[x])
                word_guess = self.dictionary[min_dist_index]
                new_transcription += word_guess + " "

            transcription = new_transcription

        if self.possibilities != None:
            distances = [self.levenshtein_distance(transcription, possibility)
                         for possibility in self.possibilities]
            min_dist_index = min(xrange(len(distances)),
                                 key=lambda x: distances[x])
            transcription_guess = self.possibilities[min_dist_index]
            transcription = transcription_guess

        return transcription
