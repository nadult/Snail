Ogolny opis dzialania:

Klient laczy sie z serwerem wysylajac informacje o rozdzielczosci, nazwie sceny.
Serwer najpierw szuka pliku o podanej nazwie w katalogu dump/ zawierajacym
wczesniej wygenerowane drzewa BVH. Jesli nie udalo mu sie znalezc, to tworzy
nowe drzewo za pomoca jednej z dwoch metod: szybkiej lub wolnej. Metoda szybka
dziala kilka razy szybciej, ale generuje drzewo gorszej jakosci dla scen
zawierajacych trojkaty o mocno roznych wielkosciach (czyli np. dla scen
architektonicznych). Serwer wczytuje takze materialy z odpowiedniego pliku .mtl
oraz teksturki jesli sa dostepne. Nastepnie serwer rozsyla te dane do kazdego z
wezlow.

Obrazek dzielony jest na kawalki, wszystkie o takiej samej wielkosci (aktualnie
16x64) ktore sa rozdzielane losowo, ale rownomiernie pomiedzy wezly renderujace.
Przydzialy te nie zmieniaja sie az do nastepnego polaczenia.

Teraz serwer, wezly i klient wchodza w glowna petle:
- Serwer odbiera konfiguracje klatki od klienta i rozsyla ja do wezlow, nastepnie
  odbiera skompresowane kawalki obrazkow od wezlow i przesyla je dalej
  do klienta. Na koniec klatki zbiera statystyki od kazdego z wezlow, sumuje
  i takze wysyla do klienta.
- Klient wysyla konfiguracje klatki (pozycja kamery, opcje, swiatla) do serwera,
  nastepnie odbiera kawalki obrazka od serwera, dekompresuje je i umieszcza pod
  odpowiednimi wspolrzednymi. Koniec klatki sygnalizowany jest pustym kawalkiem.
  Obrazek wyrzucany jest na ekran za pomoca PBO
  (http://www.songho.ca/opengl/gl_pbo.html).
- Wezly w petli pobieraja konfiguracje klatki od serwera i renderuja odpowiednie
  kawalki:

	Jesli nie dziala odpowiednia liczba watkow SPU, to tworzone sa nowe
	Dopoki sa nieobsluzone kawalki obrazka
		Wrzuc informacje o kawalku obrazka do zrenderowania do puli

	Dopoki nie obsluzono (zrenderowano, skompresowano i wyslano) wszystkich kawalkow
		Jesli zrenederowano packetSize kawalkow
			skompresuj kawalki
			wyslij kawalki
			zwieksz liczbe obsluzonych kawalkow

	Zbierz statystyki i wyslij do serwera
	
	Watki na SPU dzialaja caly czas, ew. zawieszaja sie na odpowiednich zmiennych
	warunkowych jesli w puli nie ma zadan do zrenderowania.


	Petla renderingu na SPU:
		
		Wez zadanie z puli
		Podziel dany kawalek na mniejsze kawalki o wielkosci 16x16
		Dla kazdego kawalka 16x16:
			Wygeneruj promienie biorac pod uwage aktualna konfiguracje kamery
			(Jesli wlaczony jest antialiasing to generowane jest 4x tyle promieni,
			 a nastepnie wynik jest usredniany)
			Wyznacz przeciecie pakietu promieni ze scena
			Ocieniuj pixele
			Jesli sa swiatla, to dla kazdego swiatla sprawdz ktore pixele sa w cieniu,
				a ktore nie i odpowiednio zmodyfikuj kolor pixeli
			Jesli wlaczone sa odbicia to wywolaj rekurencyjnie procedure renderujaca dla
				odpowiednio wygenerowanych promieni i zmodyfikuj kolor pixeli
			Skonwertuj kolor na R8G8B8 i zapisz w tablicach red, green, blue
		Skopiuj dane koloru z tablic red, green, blue do pamieci zewnetrznej


Szczegolowy opis BVH i roznych algorytmow z nim zwiazanych:
http://www.mpi-inf.mpg.de/~guenther/STAR-RTAS/index.html	
