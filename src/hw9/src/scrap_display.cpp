#include <SFML/Graphics.hpp>
#include <iostream>

using namespace std;

void loadImage(const char* pathname, sf::Texture& texture, sf::Sprite& sprite)
{
     texture.loadFromFile("./testImage.png");
     sprite.setTexture(texture);
     sprite.setTextureRect(sf::IntRect(0,0,640,480));
}

int main()
{
     cout << "Hi fren. Will you give me Jeff or Michelle's printed papers? Please press the spacebar or type \"y\" or \"n\"\n";
     sf::RenderWindow window(sf::VideoMode(640,480), "SFML works!");
     sf::Texture texture;
     sf::Sprite sprite;
     loadImage("testImage.png", texture, sprite);

     while (window.isOpen())
     {
          sf::Event event;
          while (window.pollEvent(event))
          {
               if (event.type == sf::Event::Closed)
                    window.close();

               if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Space)
               {
                   window.close();
                   cout << "Oh hey\n";
               }

               else if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Y)
               {
                   window.close();
                   cout << "Thanks for the printed paper\n";
               }

               else if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::N)
               {
                   window.close();
                   cout << "I am sad, but I will wait for another person to give me the paper\n";
               }
          }

          window.clear();
          window.draw(sprite);
          window.display();
     }

     return 0;

}
