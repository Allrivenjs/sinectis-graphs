package Algorith

import (
	"fmt"
	"sinectis-graphs/Graph"
)

type DegreeCounts map[string]int

type IncidenceMatrix map[string][]int

type DegreeCountsString map[string]string

type Path struct {
	Vertices []string
	Length   float64
}

type Html struct {
	Table string `json:"table"`
}

func (h *Html) GenerateTableIncidenceMatrix(edges *Graph.Edges, incidenceMatrix IncidenceMatrix) {
	// Generar la tabla HTML
	html := "<table border=\"1\" cellspacing=\"0\" cellpadding=\"5\">"
	html += "<tr><th></th>"

	for key := range *edges {
		html += "<th>a" + fmt.Sprint(key+1) + "</th>"
	}

	html += "</tr>"

	for node, row := range incidenceMatrix {
		html += "<tr><td><strong>" + node + "</strong></td>"
		for _, value := range row {
			html += "<td>" + fmt.Sprint(value) + "</td>"
		}
		html += "</tr>"
	}

	html += "</table>"

	// Imprimir la tabla HTML
	//fmt.Println(html)
	h.Table = html
}

func (incidenceMatrix *IncidenceMatrix) CalculateDegreeCounts() DegreeCountsString {
	degreeCounts := make(DegreeCounts)
	degreeCountsString := make(DegreeCountsString)
	for node, row := range *incidenceMatrix {
		degreeCounts[node] = Sum(row)
	}
	// Imprimir la lista de vértices y la cantidad de grados de cada vértice
	//fmt.Println("Lista de vértices y grados:\n")
	for node, degree := range degreeCounts {
		degreeCountsString[node] = fmt.Sprintf("G(%s) = %d", node, degree)
	}
	return degreeCountsString

}
