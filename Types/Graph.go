package Types

import (
	"fmt"
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

func (nodes *Nodes) GetIncidenceMatrix(edges *Edges) IncidenceMatrix {
	incidenceMatrix := make(IncidenceMatrix, len(*nodes))
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

func (nodes *Nodes) UniqueStringsAndSort() Nodes {
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
