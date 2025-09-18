# Wordly Game

A simple and fun **GUI-based Wordly** game with multiple difficulty levels.  
You can choose between **3, 4, 5, or 6-letter words** (default is 5 letters).  

The words are loaded **offline** from `.txt` files, each containing **5,000 words** for variety.

---

## 🎮 Features
- **Multiple difficulty levels**: 3, 4, 5, or 6-letter words.
- **Default mode**: 5-letter words.
- **Offline play**: Words are stored locally in `.txt` files.
- **Large word pool**: 5,000 words per file for endless gameplay.
- **User-friendly GUI** interface that highlights letters based on their correctness.

---

## 🛠 Approach
Initially, the game used an **online API** to fetch words.  
However:
- The API was limited to **1,000 words** without upgrading.
- It required an **active internet connection**.

To make the game **faster, more reliable, and fully offline**, the word lists were moved to local `.txt` files.  
Each `.txt` file contains **5,000 words** for the chosen difficulty level.  
This approach allows:
- **Unlimited gameplay** without relying on external services.
- Better **performance** in offline conditions.
- Easy difficulty selection in the GUI.

The GUI highlights each letter:
- **Green** if the letter is correct and in the correct position.
- **Yellow** if the letter exists but in a different position.
- **Gray** if the letter is not in the word.

---

## 📂 Project Structure
