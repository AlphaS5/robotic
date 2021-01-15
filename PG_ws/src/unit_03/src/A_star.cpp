declare openlist as PriorityQueue with Nodes // Prioritätenwarteschlange for the grey shit
declare closedlist as Set with Nodes //For the black ones




program a-star
    // Initialisierung der Open List, die Closed List ist noch leer
    // (die Priorität bzw. der f-Wert des Startknotens ist unerheblich)
    openlist.enqueue(startknoten, 0);
    // diese Schleife wird durchlaufen bis entweder
    // - die optimale Lösung gefunden wurde oder
    // - feststeht, dass keine Lösung existiert
    repeat
        // Knoten mit dem geringsten f-Wert aus der Open List entfernen
        currentNode := openlist.removeMin()
        // Wurde das Ziel gefunden?
        if currentNode == zielknoten then
            return PathFound
        // Der aktuelle Knoten soll durch nachfolgende Funktionen
        // nicht weiter untersucht werden, damit keine Zyklen entstehen
        closedlist.add(currentNode)
        // Wenn das Ziel noch nicht gefunden wurde: Nachfolgeknoten
        // des aktuellen Knotens auf die Open List setzen
        expandNode(currentNode)
    until openlist.isEmpty()
    // die Open List ist leer, es existiert kein Pfad zum Ziel
    return NoPathFound
end
// überprüft alle Nachfolgeknoten und fügt sie der Open List hinzu, wenn entweder
// - der Nachfolgeknoten zum ersten Mal gefunden wird, oder
// - ein besserer Weg zu diesem Knoten gefunden wird
function expandNode(currentNode)
    foreach successor of currentNode
        // wenn der Nachfolgeknoten bereits auf der Closed List ist – tue nichts
        if closedlist.contains(successor) then
            continue
        // g-Wert für den neuen Weg berechnen: g-Wert des Vorgängers plus
        // die Kosten der gerade benutzten Kante
        tentative_g = g(currentNode) + c(currentNode, successor)
        // wenn der Nachfolgeknoten bereits auf der Open List ist,
        // aber der neue Weg nicht besser ist als der alte – tue nichts
        if openlist.contains(successor) and tentative_g >= g(successor) then
            continue
        // Vorgängerzeiger setzen und g Wert merken oder anpassen
        successor.predecessor := currentNode
        g(successor) = tentative_g
        // f-Wert des Knotens in der Open List aktualisieren
        // bzw. Knoten mit f-Wert in die Open List einfügen
        f := tentative_g + h(successor)
        if openlist.contains(successor) then
            openlist.updateKey(successor, f)
        else
            openlist.enqueue(successor, f)
    end
end
