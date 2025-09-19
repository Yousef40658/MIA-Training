+++#include <iostream>
#include <vector>
#include <functional>
#include <cstdlib>
#include <ctime>
#include <thread>
#include <chrono>

using namespace std;

///-----------
// Parent
//------------
class AutoBot {
public:                         //accessible from outside
    static bool megatron_move;  // turn controlling
    
    int health;
    double accuracy;
    int damage;

    //initialized values
    AutoBot() {
        health = 100;
        accuracy = 0.0;
        damage = 0;
    }


    ///Methods

    //placeholder -> overridden by children
    virtual void attack() {}

    void choose_attack_move(string name, vector<pair<string,function<void()>>> &moves_list) {
        //pick random move
        int random_index = rand() % moves_list.size();
        auto chosen_move = moves_list[random_index];

        //execute
        chosen_move.second();

        //random chance
        double chance = (double)rand() / RAND_MAX;
        if (chance <= accuracy) {
            cout << name << " attacked with " << chosen_move.first << " !" << endl;
        } else {
            cout << name << " missed " << chosen_move.first << " !" << endl;
            damage = 0;
        }
    }
};

//------------
// Children 
//------------
class Optimus : public AutoBot {

public:
    vector<pair<string,function<void()>>> moves;

    //Fill moves with lambada commands so they can be run 
    Optimus() {
        moves = {
            {"ion_rifle",     [this]() { ion_rifle(); }},
            {"Energon_swords",[this]() { Energon_swords(); }},
            {"shoulder_canon",[this]() { shoulder_canon(); }}
        };
    } 

    void ion_rifle() {
        damage = 6;
        accuracy = 1.0;
    }

    void Energon_swords() {
        damage = 12;
        accuracy = 0.8;
    }

    void shoulder_canon() {
        damage = 45;
        accuracy = 0.25;
    }

    void attack() override {
        choose_attack_move("Optimus", moves);
    }
};

//----------------------------------------------------

class Megatron : public AutoBot {
public:
    vector<pair<string,function<void()>>> moves;

    Megatron() {
        moves = {
            {"Fusion_cannon",[this]() { Fusion_cannon(); }},
            {"Fusion_sword", [this]() { Fusion_sword(); }},
            {"Tank_mode",    [this]() { Tank_mode(); }}
        };
    }

    void Fusion_cannon() {
        damage = 9;
        accuracy = 0.9;
    }

    void Fusion_sword() {
        damage = 18;
        accuracy = 0.7;
    }

    void Tank_mode() {
        damage = 60;
        accuracy = 0.15;
    }

    void attack() override {
        choose_attack_move("Megatron", moves);
    }
};

//-----------
//Game_logic
//-----------

bool AutoBot::megatron_move = true;

int main() {
    
    srand(time(0));

    Optimus optimus;
    Megatron megatron;

    int Turn = 1;

    while (optimus.health > 0 && megatron.health > 0) {
        cout << "Turn " << Turn << endl;

        if (AutoBot::megatron_move) {
            megatron.attack();
            optimus.health -= megatron.damage;
            if (optimus.health < 0) optimus.health = 0;
        } else {
            optimus.attack();
            megatron.health -= optimus.damage;
            if (megatron.health < 0) megatron.health = 0;
        }

        AutoBot::megatron_move = !AutoBot::megatron_move;

        cout << "Health of Optimus     : " << optimus.health << endl;
        cout << "Health of Megatron    : " << megatron.health << endl;
        cout << "\n-------------------------------------\n";
        Turn++;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (optimus.health > 0) {
        cout << "OPTIMUS WONNN!!" << endl;
    } else {
        cout << "Evil has won T-T " << endl;
    }

    return 0;
}
