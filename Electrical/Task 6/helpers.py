import random 


def letters_frequency(word) :
    freq = {}
    for ch in word :
        freq[ch] = freq.get(ch, 0) + 1
    return freq