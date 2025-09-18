import random
import requests
import os
from enum import Enum
import helpers

#enum for more readability , returns colour to automate it a bit in Gui
class flow(Enum):
    correct_letter_in_place = "#6aaa64"
    correct_letter_not_in_place = "#c9b458"
    wrong_letter ="#787c7e"


class Wordle :
    def __init__(self, word_length = 5 , max_attempts = 6):
        self.word_length = word_length 
        self.max_attempts = max_attempts
        self.selected_word = self.select_word(word_length)
        print("Selected word:", self.selected_word)
        
        #Flow
        self.Won = False
        self.game_over = False
        self.attempts_left = max_attempts

    def select_word(self,word_length) :                               #passed as a parameter to reduce dependency 
        file_name = f'{word_length}-letter-words.txt'                 #3-letter-words.txt , 4-letter-words.txt
        folder_path = os.path.dirname(os.path.abspath(__file__))      
        file_path = os.path.join(folder_path,file_name)               #full_path -> no need to run from the folder place
        
        #reading file
        with open(file_path) as f :                                   #storing all words in memory to fetch from it instead of file -> faster
            self.words = f.read().splitlines()                        #splits by \n default , self. to be used by other methods
        self.n_of_words = len(self.words)                             #much more words than API -> even bigger datasets can be used

        #picking a random word
        selected_word_index = random.randint(0,self.n_of_words - 1)
        return self.words[selected_word_index]
    

    #evaluation
    def evaluate_guess(self,guess) :
        #we'll allow entering only when they've the same number of charcaters in gui
        letter_eval = []    
        index = 0                                                                       #to find duplicate letters

        # normalize inputs for reliable comparisons
        guess = guess.lower()
        target = self.selected_word.lower()

        # frequency dict (helpers should return lowercase counts or accept lowercase input)
        freq_matching = helpers.letters_frequency(target)

        #if the correct word
        if guess == target :
            self.Won = True                                                             #game_won
            self.game_over = True
            letter_eval = [flow.correct_letter_in_place] * self.word_length             #returs full list of correct letters
            return letter_eval
        
        #
        elif guess not in self.words :                              #if the entered word not in txt
            return "Word doesn't exist in game , try again"
        
        #slots so we can assign by index
        letter_eval = [None] * self.word_length

        for index in range(self.word_length):
            ch = guess[index]
            if ch == target[index]:                                 #correct letter in index
                letter_eval[index] = flow.correct_letter_in_place   
                freq_matching[ch] = freq_matching.get(ch, 0) - 1    #decrease the frequency of the letter

        
        for index in range(self.word_length):                       
            if letter_eval[index] is None:                          #not green -not in correct index-
                ch = guess[index]
                if freq_matching.get(ch, 0) > 0:                    #if the letter exists reutnrs its frequency
                    letter_eval[index] = flow.correct_letter_not_in_place
                    freq_matching[ch] -= 1
                else:                                               #doesn't exist
                    letter_eval[index] = flow.wrong_letter

        self.attempts_left -= 1  
        if self.attempts_left == 0 :                                #game_ends when there isn o more attempts
            self.game_over = True
        
        return letter_eval 
