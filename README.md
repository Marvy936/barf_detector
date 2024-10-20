# barf_detector

https://chatgpt.com/c/671505e4-0f48-8012-ad5e-d82fbad8a79b

Cieľom projektu je vytvoriť detektor štekotu psa pomocou Raspberry Pi Zero WH, ktorý bude odosielať upozornenia cez Pushbullet v prípade detekcie hlasitého zvuku (štekotu). Detektor bude aktívny len vtedy, keď nie ste doma, čo určíme detekciou zamknutia dverí.

Rozhodli sme sa pre nasledujúce komponenty:

    KY-037 zvukový senzor:
        Deteguje hlasitý zvuk prekračujúci nastavenú prahovú úroveň.
        Jednoduchý na použitie a vhodný pre náš účel, keďže nepotrebujeme analyzovať typ zvuku.

    KY-024 Hallov senzor s N52 magnetom:
        Slúži na detekciu stavu zamknutia dverí.
        Bezkontaktné a spoľahlivé riešenie bez zásahu do mechaniky zámku.

    4-kanálový logický konvertor 5V/3.3V:
        Použijeme ho na zníženie napäťových úrovní z 5V na 3.3V medzi senzormi a Raspberry Pi.
        Rozhodli sme sa použiť 4-kanálový konvertor, pretože ho už máte k dispozícii. Poskytuje aj možnosť budúceho rozšírenia projektu o ďalšie senzory alebo zariadenia bez potreby ďalšieho konvertora.

Jednotný postup krok za krokom
Krok 1: Zoznam potrebných komponentov

    Hardvér:
        Raspberry Pi Zero WH s GPIO headerom.
        MicroSD karta (minimálne 8 GB).
        Napájací adaptér pre Raspberry Pi (5V, minimálne 2A).
        KY-037 zvukový senzor.
        KY-024 Hallov senzor.
        N52 neodýmový magnet.
        4-kanálový logický konvertor 5V/3.3V.
        Káble na prepojenie (dupont káble).
        Voliteľne: držiaky, lepidlo alebo obojstranná páska pre upevnenie senzorov.

    Softvér:
        Raspberry Pi OS Lite (bez grafického rozhrania).
        Python 3 a potrebné knižnice (RPi.GPIO, pushbullet.py).

Krok 2: Príprava Raspberry Pi

    Inštalácia Raspberry Pi OS:

        Stiahnite si najnovšiu verziu Raspberry Pi OS Lite z oficiálnej stránky.

        Použite nástroj ako balenaEtcher na napálenie obrazu na microSD kartu.

        V adresári /boot na SD karte vytvorte prázdny súbor s názvom ssh, aby ste povolili vzdialený prístup cez SSH.

        Ak používate Wi-Fi, vytvorte v /boot súbor wpa_supplicant.conf s nasledujúcim obsahom (upravte podľa vašej siete):

        makefile

    country=SK
    ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
    update_config=1

    network={
        ssid="NazovVasejSiete"
        psk="HesloVasejSiete"
    }

Spustenie Raspberry Pi:

    Vložte SD kartu do Raspberry Pi a zapnite ho.

    Zistite IP adresu Raspberry Pi (napr. cez router alebo nástroj Advanced IP Scanner).

    Pripojte sa cez SSH:

    bash

    ssh pi@<IP_adresa_Raspberry_Pi>

    Predvolené heslo je raspberry.

Aktualizácia systému:

bash

    sudo apt-get update
    sudo apt-get upgrade

Krok 3: Pripojenie KY-037 zvukového senzora pomocou 4-kanálového logického konvertora

Bezpečnostná poznámka: Raspberry Pi GPIO piny pracujú na 3.3V logike. Priame pripojenie 5V výstupu senzora môže poškodiť Raspberry Pi. Použijeme preto 4-kanálový logický konvertor, ktorý už máte k dispozícii.

    Zapojenie logického konvertora:
        Napájanie logického konvertora:
            HV (High Voltage): Pripojte k 5V pinu Raspberry Pi.
            LV (Low Voltage): Pripojte k 3.3V pinu Raspberry Pi.
            GND (zem): Spojte oba GND piny (HV a LV) a pripojte ich k GND pinu Raspberry Pi.

    Zapojenie KY-037 senzora:
        KY-037 piny:
            VCC: Pripojte k 5V pinu Raspberry Pi.
            GND: Pripojte k GND pinu Raspberry Pi.
            D0 (digitálny výstup): Pripojte k HV1 pinu logického konvertora.
        Logický konvertor:
            LV1: Pripojte k zvolenému GPIO pinu Raspberry Pi (napr. GPIO 17).

Krok 4: Pripojenie KY-024 Hallovho senzora a N52 magnetu pomocou 4-kanálového logického konvertora

    Inštalácia magnetu:
        Prilepte N52 magnet na pohyblivú časť zámku (napr. jazýček alebo kľúč), ktorá sa pohybuje pri zamknutí.

    Umiestnenie KY-024 senzora:
        Umiestnite senzor tak, aby detegoval prítomnosť magnetu, keď sú dvere zamknuté.
        Uistite sa, že senzor nebráni v bežnom používaní dverí.

    Zapojenie KY-024 senzora:
        KY-024 piny:
            VCC: Pripojte k 5V pinu Raspberry Pi.
            GND: Pripojte k GND pinu Raspberry Pi.
            DO (digitálny výstup): Pripojte k HV2 pinu logického konvertora.
        Logický konvertor:
            LV2: Pripojte k zvolenému GPIO pinu Raspberry Pi (napr. GPIO 23).

Krok 5: Inštalácia potrebných knižníc

    RPi.GPIO pre prácu s GPIO pinmi:

    bash

sudo apt-get install python3-rpi.gpio

Pushbullet knižnica pre odosielanie upozornení:

bash

    pip3 install pushbullet.py

Krok 6: Získanie Pushbullet API kľúča

    Registrácia na Pushbullet:
        Navštívte www.pushbullet.com a vytvorte si účet.

    Získanie API kľúča:
        Vo svojom účte prejdite do Settings a skopírujte svoj Access Token.

Krok 7: Vytvorenie Python skriptu

    Vytvorte nový súbor:

    bash

nano detektor_stekotu.py

Vložte nasledujúci kód (nezabudnite nahradiť 'VÁŠ_API_KĽÚČ' svojím Pushbullet API kľúčom):

python

    import RPi.GPIO as GPIO
    import time
    from pushbullet import Pushbullet

    # Nastavenie Pushbullet
    pb = Pushbullet('VÁŠ_API_KĽÚČ')

    # Nastavenie GPIO pinov
    SOUND_SENSOR_PIN = 17  # KY-037 D0 výstup cez LV1
    LOCK_SENSOR_PIN = 23   # KY-024 DO výstup cez LV2

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SOUND_SENSOR_PIN, GPIO.IN)
    GPIO.setup(LOCK_SENSOR_PIN, GPIO.IN)

    def is_door_locked():
        # Zistite, či je magnet detegovaný
        return GPIO.input(LOCK_SENSOR_PIN) == GPIO.LOW  # Upraviť podľa testovania

    def send_notification(title, message):
        pb.push_note(title, message)

    try:
        while True:
            if is_door_locked():
                if GPIO.input(SOUND_SENSOR_PIN) == GPIO.LOW:
                    print("Hlasitý zvuk detegovaný!")
                    send_notification("Upozornenie", "Detegovaný hlasitý zvuk (pes môže štekať)!")
                    time.sleep(10)  # Prodleva, aby sa zabránilo opakovaným upozorneniam
            else:
                print("Doma, detektor je neaktívny.")
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()

    Uložte a zatvorte súbor:
        Stlačte CTRL + X, potom Y a Enter.

Krok 8: Testovanie a kalibrácia

    Kalibrácia KY-037 zvukového senzora:
        Otáčaním potenciometra na senzore nastavte prahovú úroveň hlasitosti.
        Pozorujte LED diódu na senzore; mala by sa rozsvietiť pri hlasitom zvuku (štekot psa).
        Upravte potenciometer tak, aby senzor reagoval na štekot psa, ale ignoroval bežné okolité zvuky.

    Kalibrácia KY-024 Hallovho senzora:
        Uistite sa, že senzor správne deteguje prítomnosť magnetu.
        Môže byť potrebné upraviť vzdialenosť medzi senzorom a magnetom alebo nastaviť potenciometer na senzore.
        Overte stav senzora pomocou skriptu a upravte podmienku v is_door_locked(), ak je to potrebné (zmena medzi GPIO.LOW a GPIO.HIGH).

    Testovanie skriptu:

        Spustite skript:

        bash

        python3 detektor_stekotu.py

        Simulujte zamknutie dverí (priblížte magnet k senzoru).

        Vydajte hlasitý zvuk (napr. napodobnite štekot) a sledujte, či sa odošle upozornenie.

        Overte, že keď sú dvere odomknuté (magnet nie je pri senzore), skript ignoruje zvuky.

Krok 9: Automatické spustenie skriptu pri štarte

    Vytvorte službu systemd:

    bash

sudo nano /etc/systemd/system/detektor_stekotu.service

Pridajte nasledujúci obsah:

ini

[Unit]
Description=Detektor Stekotu
After=multi-user.target

[Service]
Type=idle
ExecStart=/usr/bin/python3 /home/pi/detektor_stekotu.py

[Install]
WantedBy=multi-user.target

Uložte a zatvorte súbor.

Povoľte a spusťte službu:

bash

sudo systemctl enable detektor_stekotu.service
sudo systemctl start detektor_stekotu.service

Overte stav služby:

bash

    sudo systemctl status detektor_stekotu.service

Krok 10: Dodatočné tipy a upozornenia

    Prečo sme sa rozhodli použiť 4-kanálový logický konvertor:
        Dostupnosť: Už máte k dispozícii 4-kanálový logický konvertor, takže ho môžeme využiť bez dodatočných nákladov.
        Rozšíriteľnosť: Poskytuje možnosť pridať ďalšie senzory alebo zariadenia v budúcnosti bez potreby ďalšieho konvertora.
        Bezpečnosť: Použitie logického konvertora zabezpečuje správne napäťové úrovne a chráni Raspberry Pi pred poškodením.

    Bezpečnosť pripojení:
        Uistite sa, že všetky pripojenia sú pevné a správne izolované.
        Dávajte pozor na správne pripojenie napájania a zemnenia na oboch stranách logického konvertora (HV a LV).

    Napájanie:
        Použite kvalitný napájací adaptér pre Raspberry Pi (5V, minimálne 2A), aby ste predišli problémom s napájaním.

    Ochrana GPIO pinov:
        Použitie logického konvertora zabezpečuje, že napäťové úrovne na GPIO pinoch neprekročia 3.3V, čo chráni Raspberry Pi pred poškodením.

    Ladenie:
        Ak narazíte na problémy, pridajte do skriptu výpisy (print()), aby ste sledovali hodnoty senzorov a identifikovali možné chyby.

Záver

Vytvorili sme jednoduchý a efektívny detektor štekotu psa, ktorý funguje len vtedy, keď nie ste doma. Použitím KY-037 zvukového senzora a KY-024 Hallovho senzora s N52 magnetom sme dosiahli spoľahlivú detekciu hlasitých zvukov a stavu zamknutia dverí.

Prečo sme sa rozhodli použiť 4-kanálový logický konvertor:

    Dostupnosť: Keďže už máte 4-kanálový logický konvertor, bolo logické ho využiť v projekte.
    Rozšíriteľnosť: Poskytuje možnosť pridať ďalšie senzory alebo moduly v budúcnosti bez potreby ďalších konvertorov.
    Bezpečnosť a spoľahlivosť: Zabezpečuje správnu logickú úroveň signálov medzi 5V senzormi a 3.3V GPIO piny Raspberry Pi, čím chráni zariadenie pred poškodením.


    Samozrejme, rád ti poskytnem schému zapojenia pre tvoj projekt. Nižšie nájdeš podrobný popis zapojenia všetkých komponentov k Raspberry Pi Zero WH pomocou 4-kanálového logického konvertora.
Schéma zapojenia komponentov k Raspberry Pi
1. Prehľad GPIO pinov Raspberry Pi Zero WH

Raspberry Pi Zero WH má 40-pinový GPIO header, usporiadaný v dvoch radoch po 20 pinov. Tu je prehľad pinov s ich funkciami:

scss

Fyzický pin (Pin #) | Funkcia
--------------------|-----------------
         1          | 3.3V
         2          | 5V
         3          | GPIO 2 (SDA1)
         4          | 5V
         5          | GPIO 3 (SCL1)
         6          | GND
         7          | GPIO 4 (GPCLK0)
         8          | GPIO 14 (TXD0)
         9          | GND
        10          | GPIO 15 (RXD0)
        11          | GPIO 17
        12          | GPIO 18 (PCM_CLK)
        13          | GPIO 27
        14          | GND
        15          | GPIO 22
        16          | GPIO 23
        17          | 3.3V
        18          | GPIO 24
        19          | GPIO 10 (MOSI)
        20          | GND
        21          | GPIO 9 (MISO)
        22          | GPIO 25
        23          | GPIO 11 (SCLK)
        24          | GPIO 8 (CE0)
        25          | GND
        26          | GPIO 7 (CE1)
        27          | ID_SD (I2C EEPROM)
        28          | ID_SC (I2C EEPROM)
        29          | GPIO 5
        30          | GND
        31          | GPIO 6
        32          | GPIO 12
        33          | GPIO 13
        34          | GND
        35          | GPIO 19
        36          | GPIO 16
        37          | GPIO 26
        38          | GPIO 20
        39          | GND
        40          | GPIO 21

2. Zapojenie 4-kanálového logického konvertora

Piny logického konvertora:

    HV (High Voltage): Napájanie 5V pre vysokonapäťovú stranu.
    LV (Low Voltage): Napájanie 3.3V pre nízkonapäťovú stranu.
    GND: Spoločná zem pre obe strany.
    HV1 - HV4: Vysokonapäťové piny pre kanály 1 až 4.
    LV1 - LV4: Nízkonapäťové piny pre kanály 1 až 4.

Zapojenie napájania:

    HV: Pripoj k 5V pinu Raspberry Pi (fyzický pin 2 alebo 4).
    LV: Pripoj k 3.3V pinu Raspberry Pi (fyzický pin 1 alebo 17).
    GND: Pripoj k GND pinu Raspberry Pi (napr. pin 6, 9, 14, 20, 25, 30, 34, 39).

3. Zapojenie KY-037 zvukového senzora

Piny KY-037:

    VCC
    GND
    A0 (analógový výstup - nepoužíva sa)
    D0 (digitálny výstup)

Zapojenie:

    VCC: Pripoj k 5V pinu Raspberry Pi (fyzický pin 2 alebo 4).
    GND: Pripoj k GND pinu Raspberry Pi (napr. pin 6).
    D0: Pripoj k HV1 pinu na logickom konvertore.

Logický konvertor pre KY-037:

    HV1: Pripoj k D0 pinu KY-037.
    LV1: Pripoj k GPIO 17 (fyzický pin 11) na Raspberry Pi.

4. Zapojenie KY-024 Hallovho senzora

Piny KY-024:

    VCC
    GND
    A0 (analógový výstup - nepoužíva sa)
    D0 (digitálny výstup)

Zapojenie:

    VCC: Pripoj k 5V pinu Raspberry Pi (fyzický pin 2 alebo 4).
    GND: Pripoj k GND pinu Raspberry Pi (napr. pin 9).
    D0: Pripoj k HV2 pinu na logickom konvertore.

Logický konvertor pre KY-024:

    HV2: Pripoj k D0 pinu KY-024.
    LV2: Pripoj k GPIO 23 (fyzický pin 16) na Raspberry Pi.

5. Kompletné zapojenie

Napájanie a zem:

    5V piny (fyzický pin 2 alebo 4):
        HV pinu logického konvertora.
        VCC pinu KY-037.
        VCC pinu KY-024.

    3.3V pin (fyzický pin 1 alebo 17):
        LV pinu logického konvertora.

    GND piny (napr. fyzický pin 6, 9, 14):
        GND pinu logického konvertora.
        GND pinu KY-037.
        GND pinu KY-024.

Logický konvertor:

    Kanál 1 (pre KY-037):
        HV1: Pripoj k D0 KY-037.
        LV1: Pripoj k GPIO 17 (fyzický pin 11).

    Kanál 2 (pre KY-024):
        HV2: Pripoj k D0 KY-024.
        LV2: Pripoj k GPIO 23 (fyzický pin 16).

    Kanály 3 a 4: Zatiaľ nevyužité, môžu byť použité v budúcnosti.

6. Zoznam použitých GPIO pinov na Raspberry Pi

    GPIO 17 (fyzický pin 11): Prijíma signál z KY-037 cez LV1 logického konvertora.
    GPIO 23 (fyzický pin 16): Prijíma signál z KY-024 cez LV2 logického konvertora.

7. Umiestnenie magnetu a senzorov

    N52 magnet:
        Prilep na pohyblivú časť zámku, ktorá sa pohybuje pri zamknutí a odomknutí.

    KY-024 Hallov senzor:
        Umiestni ho tak, aby detegoval magnet, keď sú dvere zamknuté.

    KY-037 zvukový senzor:
        Umiestni v miestnosti, kde sa pes nachádza, s čo najlepším prístupom k zvukom.

Schéma zapojenia v textovej podobe

1. KY-037 zvukový senzor:

    VCC → 5V (pin 2 alebo 4) na Raspberry Pi
    GND → GND (pin 6) na Raspberry Pi
    D0 → HV1 na logickom konvertore
    Logický konvertor HV1 ↔ LV1
    LV1 → GPIO 17 (pin 11) na Raspberry Pi

2. KY-024 Hallov senzor:

    VCC → 5V (pin 2 alebo 4) na Raspberry Pi
    GND → GND (pin 9) na Raspberry Pi
    D0 → HV2 na logickom konvertore
    Logický konvertor HV2 ↔ LV2
    LV2 → GPIO 23 (pin 16) na Raspberry Pi

3. Logický konvertor napájanie:

    HV → 5V (pin 2 alebo 4) na Raspberry Pi
    LV → 3.3V (pin 1 alebo 17) na Raspberry Pi
    GND → GND (pin 14) na Raspberry Pi

Bezpečnostné upozornenia

    Napájanie:
        Uisti sa, že HV pinu logického konvertora je pripojený k 5V, a LV pinu k 3.3V.
        Nikdy nepripájaj 5V signály priamo na GPIO piny Raspberry Pi bez použitia logického konvertora.

    GND pripojenia:
        Všetky GND piny (Raspberry Pi, logický konvertor, senzory) musia byť prepojené.

    Kontrola zapojenia:
        Pred zapnutím Raspberry Pi skontroluj všetky pripojenia, aby si predišiel skratom alebo nesprávnym zapojeniam.

Dodatočné tipy

    Použitie farebných káblov:
        Červený pre VCC (napájanie).
        Čierny pre GND (zem).
        Iné farby pre signálne piny, aby si ľahšie identifikoval pripojenia.

    Označenie pinov:
        Ak používaš nepájivé pole alebo prepojovacie káble, označ si piny alebo káble pre lepšiu orientáciu.

    Testovanie:
        Po zapojení každého komponentu ho otestuj samostatne, aby si overil správnu funkčnosť.
