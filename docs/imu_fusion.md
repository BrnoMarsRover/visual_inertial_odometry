# Fůze dat z IMU

Když jsem poprvé vyzkoušel algoritmus se zapnutou IMU fuzí, byl velmi chybový a algoritmus funguje efektivněji bez ní.

## Příprava dat

- Provedl jsem měření charakteristik šumu IMU. Použil jsem balíček [Allan ROS2](https://github.com/CruxDevStuff/allan_ros2.git). Výsledky jsou v konfiguračním souboru f450.

- Z fusion modelu byla získána přesná transformace mezi kamerou a IMU, kde frame pro IMU uvažuji `base_frame`.

- Orientace os byla upravena tak, aby odpovídala konvenci souřadnicového systému ENU.

## Let 11.02.2026 (indoor)
Let při vypnutých motorech (dron byl držen v ruce) tak aby data z IMU byla snadno vyhodnotitelná. Během letu postupně s dronem pohybuju v jednotlivých osách, nejprve pro test akcelerometru a poté pro test gyroskopu.

Nahraná data jsou vyhodnocena v matlabu skriptem `imu_fusion_compare.m`.

<figure align="center">
  <img src="images/IMU_fusion_ENABLE_timeline.png" alt="IMU Fusion ENABLE timeline">
  <figcaption><i>IMU Fusion ENABLE - timeline pozice, akcelerace a gyroskopu</i></figcaption>
</figure>
Na obrázku je možné pozorovat všechny tři osy odometrie z VIO v závislosti na čase. Při zkoušce akcelerometru se problém s trhanou odometrií nijak neprojevil. Zajímavý moment je zvýrazněný červenou čarou, při zkoušce gyroskopu v ose y došlo nezmámé chybě díky které se dojde k chybě VIO. Od té chvíle spočítaná odometrie nedává smysl.

### Závěr

- Orientace os IMU je správná (ověřeno podle obrazu z kamery).
- Z dat nelze jednoznačně určit chybu, protože v označeném okamžiku mířila kamera RealSense směrem ke stropu na světla. Je tedy možné, že algoritmus v tu chvíli pouze upřednostnil data z IMU, což se mohlo stát i v jiných okamžicích.

## Gravitační vektor

Diskuzní [fórum](https://forums.developer.nvidia.com/t/imu-fusion/316150) tvrdí že:

- Šipka gravitačního vektoru se poprvé objeví po úspěšné inicializaci (camera-to-IMU alignment), což trvá 10-20 sekund pohybu kamery. Od tohoto momentu běží IMU fúze kontinuálně.
- Šipka gravitačního vektoru se znovu objeví v momentě, kdy dojde ke ztrátě visual trackingu a algoritmus následně znovu synchronizuje (re-alignment) kameru a IMU po obnovení trackingu.