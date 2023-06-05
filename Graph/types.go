package Graph

import (
	"fmt"
	"sinectis-graphs/Algorith"
	"sort"
	"strings"
)

type Position struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

type Nodes []string

type Edge struct {
	Id       string   `json:"id"`
	Node1    string   `json:"node1"`
	Node2    string   `json:"node2"`
	Weighted string   `json:"weighted"`
	Position Position `json:"position"`
}

type Edges []Edge

type Graph struct {
	Nodes Nodes `json:"nodes"`
	Edges Edges `json:"edges"`
}

type GraphMap map[string][]string

func (edges *Edges) GetSimplePath(start string, end string) string {
	path := edges.findSimplePath(start, end)
	var SimplePath string
	//
	if path != nil {
		// Imprimir el resultado
		SimplePath += fmt.Sprintf("Camino simple\n")
		SimplePath += fmt.Sprintf("C = {" + strings.Join(path, ", ") + "}")
		// Aquí puedes agregar la longitud total del camino si proporcionas información sobre las distancias
	} else {
		SimplePath = fmt.Sprintf("No se encontró un camino simple entre las ciudades especificadas.")
	}
	return SimplePath
}

func (edges *Edges) IncidenceFunc() string {
	incidenceFunction := "Funciones de incidencia:\nf(G) = {\n"
	for _, edge := range *edges {
		incidenceFunction += "fg(" + edge.Id + ") = (" + edge.Node1 + ", " + edge.Node2 + "),\n"
	}
	incidenceFunction = strings.TrimSuffix(incidenceFunction, ",\n") + "\n}"

	// Imprimir la función de incidencia
	return incidenceFunction
}

func (nodes *Nodes) GenerateVertexList() string {
	vertexList := "Vértices: " + fmt.Sprint(len(*nodes)) + " \nV(G) = {"
	for _, node := range *nodes {
		vertexList += node + ", "
	}
	vertexList = strings.TrimSuffix(vertexList, ", ") + "}"

	// Imprimir la lista de vértices
	//fmt.Println(vertexList)
	return vertexList
}

// Función de búsqueda de camino simple
func (edges *Edges) findSimplePath(start string, end string) []string {
	graph := make(GraphMap)
	for _, edge := range *edges {
		graph[edge.Node1] = append(graph[edge.Node1], edge.Node2)
		graph[edge.Node2] = append(graph[edge.Node2], edge.Node1)
	}

	visited := make(map[string]bool)
	for vertex := range graph {
		visited[vertex] = false
	}

	var path []string

	if Algorith.Dfs(graph, start, end, visited, &path) {
		result := make([]string, len(path)-1)

		for i := 0; i < len(path)-1; i++ {
			result[i] = fmt.Sprintf("(%s, %s)", path[i], path[i+1])
		}

		return result
	}

	return nil
}

func (nodes *Nodes) getIncidenceMatrix(edges *Edges) Algorith.IncidenceMatrix {
	incidenceMatrix := make(Algorith.IncidenceMatrix, len(*nodes))
	for _, node := range *nodes {
		row := make([]int, len(*edges))
		for i, edge := range *edges {
			if edge.Node1 == node || edge.Node2 == node {
				row[i] = 1
			} else {
				row[i] = 0
			}
		}
		incidenceMatrix[node] = row
	}
	return incidenceMatrix
}

func (nodes *Nodes) uniqueStringsAndSort() Nodes {
	encountered := make(map[string]bool)
	result := make(Nodes, 0)

	for _, str := range *nodes {
		str = strings.TrimSpace(str)
		if !encountered[str] {
			encountered[str] = true
			result = append(result, str)
		}
	}
	sort.Strings(result)
	return result
}
