# PCL--Dom-Wurzen
2015-03-02, Lars

Momentan beherrscht das "Programm" foldende Methoden:
(Begriff in Klammer nennt die entsprechende Klasse, jede Methode speichert Ergebnis in Datei und gibt sie auch an Programm zurück)

- Punktwolken einlesen und Visualisieren (Reader[aus Library] +Visualizer)
- Punktwolke ausdünnen (Voxelgrid)
- Ausreißer entfernen (StatisticalOutlierRemoval)
- Bestimmung von Normalen für gesetzte Anzahl von Punkten (NormalEstimation)
- Finden von Ebenen und Abspeichern dieser mom. im .exe-ordner, + Nennen der Ebenenparameter mom. in Console,(SAGSegmentation+ExtractIndices, diese Funktion aktuell noch in der Main (Start.cpp) eingebunden)


Ideen zu weiterem Ablauf:

- ==> siehe Bernd Grafes Anleitung in Präsentation 
- schleife in main, die jeweils in einem durchlauf folgendes machen lässt:
- finde eine ebene, extrahiere die punkte (SACSegmentation)
- bereinigen (OutlierRemoval)
- Entfernen, was nicht zur Hauptgruppe gehört (Clusterextraction)
- Punkte der außenkante finden (BoundaryEstimation )
- außenkante in Linie umwandeln (ConvexHull2D / ConcaveHull)
- polygon erstellen (Ebene anhand der ebenenkoeffizienten aus SACSegmentation + Begrenzung durch Außenkante?)
- nächste Ebene finden (oder beim ersten mal alle Ebenen finden,speichern und nacheinander durch diesen prozess schicken)

- Polygone zusammensetzen und ggf verschneiden


Ideen zur Arbeitsweise:

- alle methoden als separate klassen erstellen, Main so klein und sauber wie möglich halten
- wie soll kommentiert werden? (Deutsch/Englisch; wie Bernd:?
	1. Prozessobjekt erstellen
	2. Input setzen
	3. Parameter setzen
	4. Ausführen
)


