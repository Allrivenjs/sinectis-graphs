package main

import (
	"fmt"
	"github.com/gofiber/fiber/v2"
	"github.com/sirupsen/logrus"
	"math/rand"
	"reflect"
	"regexp"
	"sort"
	"strings"
	"time"
)

type Request struct {
	Vertex   string      `json:"vertex"`
	Start    string      `json:"start"`
	End      string      `json:"end"`
	SubGraph interface{} `json:"subGraph"`
}

type Nodes []string
type Edge struct {
	Id    string `json:"id"`
	Node1 string `json:"node1"`
	Node2 string `json:"node2"`
}
type Edges []Edge
type Graph struct {
	Nodes Nodes `json:"nodes"`
	Edges Edges `json:"edges"`
}
type GraphMap map[string][]string
type Html struct {
	Table string `json:"table"`
}

type DegreeCounts map[string]int

type IncidenceMatrix map[string][]int

type DegreeCountsString map[string]string

type GraphResponse struct {
	Nodes             []string           `json:"nodes"`
	Edges             Edges              `json:"edges"`
	IncidenceMatrix   IncidenceMatrix    `json:"incidenceMatrix"`
	IncidenceFunction string             `json:"incidenceFunction"`
	VertexList        string             `json:"vertexList"`
	DegreeCounts      DegreeCountsString `json:"degreeCounts"`
	Html              Html               `json:"html"`
	SimplePath        string             `json:"simplePath"`
	SubGraph          []Graph            `json:"subGraph"`
}

func filterInput(e string) (Nodes, Edges) {
	//re := regexp.MustCompile(`/(\w+) *= *\(([^,]+), *([^)]+)\)/`)
	e = strings.ReplaceAll(e, "\n", "")
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

func startA(r Request) GraphResponse {

	// Extraer las funciones de incidencia

	// Crear un slice de aristas y un slice de nodos
	edges := make(Edges, 0)
	nodes := make(Nodes, 0)

	nodes, edges = filterInput(r.Vertex)

	//// Encuentra un camino simple entre las ciudades
	simplePath := edges.GetSimplePath(r.Start, r.End)

	// Eliminar duplicados de nodos y ordenar alfabéticamente
	nodes = nodes.uniqueStringsAndSort()

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

	graph := buildGraph(edges)
	subgraph := make([]Graph, 2)
	subGraphType := reflect.TypeOf(r.SubGraph).Kind()
	//fmt.Println(subGraphType)
	switch subGraphType {
	case reflect.Array:
		var n Nodes
		for _, node := range r.SubGraph.([]string) {
			n = append(n, node)
		}
		for i := range subgraph {
			subgraph[i] = extractSubgraph(graph, n)
		}

	case reflect.Float64:
		for i := range subgraph {
			subgraph[i] = extractRandomSubgraph(graph, int(r.SubGraph.(float64)))
		}
	}

	return GraphResponse{
		Nodes:             nodes,
		Edges:             edges,
		IncidenceMatrix:   incidenceMatrix,
		IncidenceFunction: incidenceFunction,
		VertexList:        vertexList,
		DegreeCounts:      degreeCounts,
		Html:              html,
		SimplePath:        simplePath,
		SubGraph:          subgraph,
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

func fusionVertices(graph Graph, vertexA string, vertexB string) Graph {
	// Crear un nuevo grafo para almacenar el resultado de la fusión
	mergedGraph := Graph{
		Nodes: make(Nodes, 0),
		Edges: make(Edges, 0),
	}

	// Copiar los nodos del grafo original excepto los vértices fusionados
	for _, node := range graph.Nodes {
		if node != vertexA && node != vertexB {
			mergedGraph.Nodes = append(mergedGraph.Nodes, node)
		}
	}

	// Copiar las aristas del grafo original excepto las aristas incidentes en los vértices fusionados
	for _, edge := range graph.Edges {
		if edge.Node1 != vertexA && edge.Node1 != vertexB && edge.Node2 != vertexA && edge.Node2 != vertexB {
			mergedGraph.Edges = append(mergedGraph.Edges, edge)
		}
	}

	// Crear una nueva arista que conecte los vértices fusionados en el nuevo vértice
	mergedEdge := Edge{
		Id:    "merged",
		Node1: vertexA,
		Node2: vertexB,
	}
	mergedGraph.Edges = append(mergedGraph.Edges, mergedEdge)

	// Agregar el nuevo vértice al grafo fusionado
	mergedGraph.Nodes = append(mergedGraph.Nodes, "merged")

	return mergedGraph
}

func buildGraph(edges Edges) Graph {
	graph := Graph{
		Nodes: Nodes{},
		Edges: edges,
	}

	nodeSet := make(map[string]bool)
	for _, edge := range edges {
		nodeSet[edge.Node1] = true
		nodeSet[edge.Node2] = true
	}

	for node := range nodeSet {
		graph.Nodes = append(graph.Nodes, node)
	}

	return graph
}

func extractSubgraph(graph Graph, subGraphNodes []string) Graph {
	subGraph := Graph{
		Nodes: make([]string, 0),
		Edges: make([]Edge, 0),
	}

	nodeSet := make(map[string]bool)
	for _, node := range subGraphNodes {
		nodeSet[node] = true
	}

	for _, edge := range graph.Edges {
		if nodeSet[edge.Node1] && nodeSet[edge.Node2] {
			subGraph.Edges = append(subGraph.Edges, edge)
			nodeSet[edge.Node1] = true
			nodeSet[edge.Node2] = true
		}
	}

	for node := range nodeSet {
		subGraph.Nodes = append(subGraph.Nodes, node)
	}

	return subGraph
}

func extractRandomSubgraph(graph Graph, numNodes int) Graph {
	subgraphNodes := getRandomNodes(graph, numNodes)
	fmt.Printf("nodes %v \n", subgraphNodes)
	return extractSubgraph(graph, subgraphNodes)
}

func getRandomNodes(graph Graph, numNodes int) Nodes {
	rand.Seed(time.Now().UnixNano())

	// Obtener todos los nodos disponibles en el grafo
	availableNodes := make(Nodes, len(graph.Nodes))
	copy(availableNodes, graph.Nodes)

	// Verificar si el número de nodos solicitados es mayor que el número total de nodos disponibles
	if numNodes > len(availableNodes) {
		numNodes = len(availableNodes)
	}

	// Elegir nodos aleatorios
	randomNodes := make(Nodes, 0, numNodes)
	for i := 0; i < numNodes; i++ {
		randomIndex := rand.Intn(len(availableNodes))
		randomNode := availableNodes[randomIndex]
		randomNodes = append(randomNodes, randomNode)

		// Eliminar el nodo aleatorio del conjunto de nodos disponibles
		availableNodes = append(availableNodes[:randomIndex], availableNodes[randomIndex+1:]...)
	}

	return randomNodes
}

func contains(list Nodes, element string) bool {
	for _, item := range list {
		if item == element {
			return true
		}
	}
	return false
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
		degreeCountsString[node] = fmt.Sprintf("G(%s) = %d", node, degree)
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

// Calcular la cantidad de grados de cada vértice
func calculateDegree(graph GraphMap) map[string]int {
	degree := make(map[string]int)

	for node, neighbors := range graph {
		degree[node] = len(neighbors)
	}

	return degree
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
func dfs(graph GraphMap, start string, end string, visited map[string]bool, path *[]string) bool {
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
