# PCL--Dom-Wurzen
2015-02-17, Lars

Momentan beherrscht das "Programm" foldende Methoden:
(Begriff in Klammer nennt die entsprechende Klasse, jede Methode speichert Ergebnis in Datei und gibt sie auch an Programm zurück)

- Punktwolken einlesen und Visualisieren (Reader[aus Library] +Visualizer)
- Punktwolke ausdünnen (Voxelgrid)
- Ausreißer entfernen (StatisticalOutlierRemoval)
- Bestimmung von Normalen für gesetzte Anzahl von Punkten (NormalEstimation)
- Finden von Ebenen und Abspeichern dieser mom. im .exe-ordner, + Nennen der Ebenenparameter mom. in Console,(SAGSegmentation+ExtractIndices, diese Funktion aktuell noch in der Main (Start.cpp) eingebunden)


in der Main (Start.cpp) sind außerdem folgende Ideen zu weiterem Ablauf notiert:

- spielen mit Treshhold und %, bis zu denen iteriert werden soll (SAGSegmentation+ExtractIndices)
- ausreißerfilter nach dem ebenenfinden ist sicher sinnvoll, da fremde objekte in ebenen liegen
- oder evtl methode die sicherstellt, dass ebene aus einem objekt besteht = segmentierung?
- es muss aus der Ebenenpunktwolke noch ein polygon erstellt werden (coefficients rausschreiben)
--> äußerste Punkte der ebene als "außenkante" finden. diese könnte als begrenzung für polygon dienen, wenn es es aus koeffizienten generiert wird
==> siehe Bernd Grafes Anleitung in Präsentation 
- schleife in main, die jeweils in einem durchlauf folgendes machen lässt:
- finde eine ebene, extrahiere die punkte 
- bereinigen und außenkante finden
- polygon erstellen (+bearbeiten? und speichern)
- nächste Ebene finden
--> alle methoden als separate klassen erstellen?

