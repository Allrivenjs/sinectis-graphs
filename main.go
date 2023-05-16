package main

import (
	"fmt"
	"github.com/gofiber/fiber/v2"
	"github.com/sirupsen/logrus"
	"regexp"
	"sort"
	"strings"
)

type Request struct {
	Vertex string `json:"vertex"`
	Start  string `json:"start"`
	End    string `json:"end"`
}

type Nodes []string
type Edge struct {
	Id    string `json:"id"`
	Node1 string `json:"node1"`
	Node2 string `json:"node2"`
}
type Edges []Edge

type Html struct {
	Table string `json:"table"`
}

type DegreeCounts map[string]int

type IncidenceMatrix map[string][]int

type DegreeCountsString map[string]string

type Graph struct {
	Nodes             []string           `json:"nodes"`
	Edges             Edges              `json:"edges"`
	IncidenceMatrix   IncidenceMatrix    `json:"incidenceMatrix"`
	IncidenceFunction string             `json:"incidenceFunction"`
	VertexList        string             `json:"vertexList"`
	DegreeCounts      DegreeCountsString `json:"degreeCounts"`
	Html              Html               `json:"html"`
	SimplePath        string             `json:"simplePath"`
}

func filterInput(e string) (Nodes, Edges) {
	//re := regexp.MustCompile(`/(\w+) *= *\(([^,]+), *([^)]+)\)/`)
	re := regexp.MustCompile(`(\w+)\s*=\s*\(([^,]+),([^)]+)\)`)
	matches := re.FindAllStringSubmatch(e, -1)
	// Crear un slice de aristas y un slice de nodos
	var edges Edges
	nodes := make(Nodes, 0)
	for _, match := range matches {
		edge := Edge{
			Id:    strings.TrimSpace(match[1]),
			Node1: strings.TrimSpace(match[2]),
			Node2: strings.TrimSpace(match[3]),
		}
		edges = append(edges, edge)
		nodes = append(nodes, strings.TrimSpace(match[2]), strings.TrimSpace(match[3]))
	}
	return nodes, edges
}

func startA(r Request) Graph {

	// Extraer las funciones de incidencia

	// Crear un slice de aristas y un slice de nodos
	edges := make(Edges, 0)
	nodes := make(Nodes, 0)

	nodes, edges = filterInput(r.Vertex)

	//// Encuentra un camino simple entre las ciudades
	simplePath := edges.GetSimplePath(r.Start, r.End)

	// Eliminar duplicados de nodos y ordenar alfabéticamente
	nodes.uniqueStringsAndSort()

	// Crear matriz de incidencia
	incidenceMatrix := nodes.getIncidenceMatrix(&edges)

	// Imprimir la matriz de incidencia
	//for node, row := range incidenceMatrix {
	//	fmt.Printf("%s: %v\n", node, row)
	//}

	var html Html

	html.GenerateTableIncidenceMatrix(&edges, incidenceMatrix)

	// Generar la lista de vértices
	vertexList := nodes.GenerateVertexList()
	//fmt.Println(vertexList)

	// Generar la función de incidencia
	incidenceFunction := edges.IncidenceFunc()
	//fmt.Println(incidenceFunction)

	// Calcular la cantidad de grados de cada vértice

	degreeCounts := incidenceMatrix.CalculateDegreeCounts()
	//fmt.Println(degreeCounts)

	return Graph{
		Nodes:             nodes,
		Edges:             edges,
		IncidenceMatrix:   incidenceMatrix,
		IncidenceFunction: incidenceFunction,
		VertexList:        vertexList,
		DegreeCounts:      degreeCounts,
		Html:              html,
		SimplePath:        simplePath,
	}
}

func main() {
	//logrus.Infof("beginning application")
	//err := godotenv.Load()
	//if err != nil {
	//	log.Fatalf("err loading: %v", err)
	//}
	//// Inicializa la base de datos
	//global.Load(c.InitDB())
	app := fiber.New()
	app.Get("/", func(c *fiber.Ctx) error {
		return c.SendString("Hello, World!")
	})

	app.Get("/graph", func(c *fiber.Ctx) error {
		var r Request
		err := c.BodyParser(&r)
		if err != nil {
			return err
		}
		return c.JSON(startA(r))
	})

	//routes.SetupRoutes(app)
	//app.Use(middlewares.RouteLogger(app))
	logrus.Fatal(app.Listen(":3000"))
}

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

func (incidenceMatrix *IncidenceMatrix) CalculateDegreeCounts() DegreeCountsString {
	degreeCounts := make(DegreeCounts)
	degreeCountsString := make(DegreeCountsString)
	for node, row := range *incidenceMatrix {
		degreeCounts[node] = sum(row)
	}
	// Imprimir la lista de vértices y la cantidad de grados de cada vértice
	//fmt.Println("Lista de vértices y grados:\n")
	for node, degree := range degreeCounts {
		degreeCountsString[node] = fmt.Sprintf("G(%s) = %d\n", node, degree)
	}
	return degreeCountsString

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

func (h *Html) GenerateTableIncidenceMatrix(edges *Edges, incidenceMatrix IncidenceMatrix) {
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

func (nodes *Nodes) getIncidenceMatrix(edges *Edges) IncidenceMatrix {
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

func (nodes *Nodes) uniqueStringsAndSort() *Nodes {
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
	return &result
}

// Calcular la cantidad de grados de cada vértice
func calculateDegree(graph map[string][]string) map[string]int {
	degree := make(map[string]int)

	for node, neighbors := range graph {
		degree[node] = len(neighbors)
	}

	return degree
}

// Función de búsqueda de camino simple
func (edges *Edges) findSimplePath(start string, end string) []string {
	graph := make(map[string][]string)

	for _, edge := range *edges {
		node1 := edge.Node1
		node2 := edge.Node2
		graph[node1] = append(graph[node1], node2)
		graph[node2] = append(graph[node2], node1)
	}

	visited := make(map[string]bool)
	for vertex := range graph {
		visited[vertex] = false
	}

	var path []string

	if dfs(graph, start, end, visited, &path) {
		result := make([]string, len(path)-1)

		for i := 0; i < len(path)-1; i++ {
			result[i] = fmt.Sprintf("(%s, %s)", path[i], path[i+1])
		}

		return result
	}

	return nil
}

func sum(arr []int) int {
	result := 0
	for _, val := range arr {
		result += val
	}
	return result
}

// Función de búsqueda en profundidad (DFS)
func dfs(graph map[string][]string, start string, end string, visited map[string]bool, path *[]string) bool {
	visited[start] = true
	*path = append(*path, start)

	if start == end {
		return true
	}

	for _, neighbor := range graph[start] {
		if !visited[neighbor] {
			if dfs(graph, neighbor, end, visited, path) {
				return true
			}
		}
	}

	*path = (*path)[:len(*path)-1]
	return false
}
