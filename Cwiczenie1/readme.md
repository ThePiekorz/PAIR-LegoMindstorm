CEL: Zaprogramowanie robota tak, aby jechał wzdłuż krawędzi czarnej linii i zatrzymał się po dojechaniu do czerwonego pola.

TABELA Z WYNIKAMI

TYP | Prędkość | Zmienne | Czas |
--- | --- | --- | --- |
Dwustanowy | 50 | Alfa = 400 | 45s |
Proporcjonalny | 250 | K = 15 | 43s |
PID V1 | 275 | K = 12, Ki = 0.2, Kd = 5 | 38s |
PID V2 | 350 | K = 12, Ki = 0.4, Kd = 5 | 30s |

Wnioski:
Po przetestowaniu kodu, okazało się, że każdy kolejny regulator miał tendencję spadkową w czasach przejazdu. 
Różnica czasu przejazdu między regulatorem dwustanowym a PID wynosiła aż 15s.
Możliwe było osiągnięcie jeszcze lepszych czasów zmieniając parametr kompensujący uchyb w przeszłości, dzięki czemu robot nie wypadał z toru i można było zwiększyć prędkość, jak widać w przypadkach
PID V1 i V2 (8s).
