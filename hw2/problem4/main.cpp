#include <iostream>
// Guessing game with "Correct", "Higher", or "Lower"

int main(){
    // Generate random number 0-99
    srand(time(0));
    int number = rand() % 100;
    int guess = -1;
    while (guess != number){
        std::cout << "Enter your guess: ";
        std::cin >> guess;
        if (guess == number){
            std::cout << "Correct!" << std::endl;
        } else if (guess < number){
            std::cout << "Higher" << std::endl;
        } else {
            std::cout << "Lower" << std::endl;
        }
    }

    return 0;
}