package main

import (
	"fmt"
	"github.com/gofiber/fiber/v2"
	"github.com/sirupsen/logrus"
	"math"
	"math/rand"
	"reflect"
	"regexp"
	"sort"
	"strconv"
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
	Id       string `json:"id"`
	Node1    string `json:"node1"`
	Node2    string `json:"node2"`
	Weighted string `json:"weighted"`
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
	Graph             Graph              `json:"graph"`
	IncidenceMatrix   IncidenceMatrix    `json:"incidenceMatrix"`
	IncidenceFunction string             `json:"incidenceFunction"`
	VertexList        string             `json:"vertexList"`
	DegreeCounts      DegreeCountsString `json:"degreeCounts"`
	Html              Html               `json:"html"`
	SimplePath        string             `json:"simplePath"`
	SubGraph          []Graph            `json:"subGraph"`
}

type Path struct {
	Vertices []string
	Length   float64
}

func filterInput(e string) (Nodes, Edges) {
	//re := regexp.MustCompile(`/(\w+) *= *\(([^,]+), *([^)]+)\)/`)
	e = strings.ReplaceAll(e, "\n", "")
	//re := regexp.MustCompile(`(\w+)\s*=\s*\(([^,]+),([^)]+)\)`)
	re := regexp.MustCompile(`(\w+)\s*=\s*\(([^,]+),([^)]+)\)\s*\[([^]]+)\]`)
	matches := re.FindAllStringSubmatch(e, -1)
	// Crear un slice de aristas y un slice de nodos
	var edges Edges
	nodes := make(Nodes, 0)
	logrus.Println(matches)
	for _, match := range matches {
		edge := Edge{
			Id:       strings.TrimSpace(match[1]),
			Node1:    strings.TrimSpace(match[2]),
			Node2:    strings.TrimSpace(match[3]),
			Weighted: strings.TrimSpace(match[4]),
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
		Graph:             graph,
		IncidenceMatrix:   incidenceMatrix,
		IncidenceFunction: incidenceFunction,
		VertexList:        vertexList,
		DegreeCounts:      degreeCounts,
		Html:              html,
		SimplePath:        simplePath,
		SubGraph:          subgraph,
	}
}

type RequestGraph struct {
	Graph Graph `json:"graph"`
}

type RequestLength struct {
	RequestGraph
	Start string `json:"start"`
	End   string `json:"end"`
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
		if r == (Request{}) {
			return c.SendString("No se recibió ningún dato")
		}
		return c.JSON(startA(r))
	})

	app.Post("/distance-matrix", func(c *fiber.Ctx) error {
		var request RequestGraph
		err := c.BodyParser(&request)
		if err != nil {
			return err
		}
		if len(request.Graph.Nodes) == 0 {
			// El campo request está vacío
			return c.Status(fiber.StatusBadRequest).JSON(fiber.Map{
				"message": "El campo 'request' no puede estar vacío",
			})
		}

		matrix := distanceMatrix(request.Graph)
		tableHTML := formatMatrixAsHTMLTable(matrix, request.Graph.Nodes)

		return c.SendString(tableHTML)
	})

	app.Post("/length", func(c *fiber.Ctx) error {
		var request RequestLength
		err := c.BodyParser(&request)
		if err != nil {
			return err
		}
		if len(request.Graph.Nodes) == 0 {
			// El campo request está vacío
			return c.Status(fiber.StatusBadRequest).JSON(fiber.Map{
				"message": "El campo 'request' no puede estar vacío",
			})
		}

		excentricity := findAllExcentricity(request.Graph, request.Start)

		length := getAllPaths(request.Graph, request.Start, request.End)
		return c.JSON(fiber.Map{
			"length":       length,
			"excentricity": excentricity,
		})
	})

	//routes.SetupRoutes(app)
	//app.Use(middlewares.RouteLogger(app))
	logrus.Fatal(app.Listen(":3000"))
}

func parseFloatWeighted(edge Edge) (float64, error) {
	weightedStr := strings.ReplaceAll(edge.Weighted, ",", ".")
	return strconv.ParseFloat(weightedStr, 64)
}

func findAllExcentricity(graph Graph, vertex string) []string {
	var excentricities []string
	var maxExcentricity float64
	var maxVertex string

	for _, node := range graph.Nodes {
		excentricity := calculateExcentricity(graph, vertex, node)
		if excentricity > maxExcentricity {
			maxExcentricity = excentricity
			maxVertex = vertex
		}
		result := formatExcentricity(node, vertex, excentricity)
		excentricities = append(excentricities, result)
	}

	result := fmt.Sprintf("La Distancia más alta que sería D (%s) cuyo valor es de %.2f el cual es el valor de excentricidad del vértice %s, E(%s) = %.2f.",
		maxVertex, maxExcentricity, maxVertex, maxVertex, maxExcentricity)
	excentricities = append(excentricities, result)

	return excentricities
}

func formatExcentricity(node string, vertex string, excentricity float64) string {
	result := fmt.Sprintf("E(%s,%s) = %.2f", vertex, node, excentricity)
	return result
}

func calculateExcentricity(graph Graph, vertex, node string) float64 {
	distance := calculateShortestDistance(graph, vertex, node)
	return distance
}

func calculateShortestDistance(graph Graph, start, end string) float64 {
	distances := make(map[string]float64)
	for _, node := range graph.Nodes {
		distances[node] = math.Inf(1)
	}
	distances[start] = 0

	queue := []string{start}

	for len(queue) > 0 {
		current := queue[0]
		queue = queue[1:]

		if current == end {
			return distances[current]
		}

		for _, edge := range graph.Edges {
			if edge.Node1 == current {
				neighbor := edge.Node2
				weight, err := parseFloatWeighted(edge)
				if err != nil {
					// Manejar el error si la conversión falla
					logrus.Fatal(err)
					// Puedes optar por omitir esta arista o asignar un valor predeterminado
					continue
				}
				alt := distances[current] + weight
				if alt < distances[neighbor] {
					distances[neighbor] = alt
					queue = append(queue, neighbor)
				}
			} else if edge.Node2 == current {
				neighbor := edge.Node1
				weight, err := parseFloatWeighted(edge)
				if err != nil {
					// Manejar el error si la conversión falla
					logrus.Fatal(err)
					// Puedes optar por omitir esta arista o asignar un valor predeterminado
					continue
				}
				alt := distances[current] + weight
				if alt < distances[neighbor] {
					distances[neighbor] = alt
					queue = append(queue, neighbor)
				}
			}
		}
	}

	return math.Inf(1)
}

func getAllPaths(graph Graph, start, end string) []string {
	visited := make(map[string]bool)
	var currentPath []string
	var allPaths []string
	var minLength = math.Inf(1)

	findAllPaths(graph, start, end, visited, currentPath, &allPaths, &minLength)
	index = 0

	for _, path := range allPaths {
		if length := getPathLengthFromFormat(path); length < minLength {
			minLength = length
		}
	}

	minLengthString := fmt.Sprintf("D(%s, %s) = min (CS) = %.2f KM", start, end, minLength)
	allPaths = append(allPaths, minLengthString)

	return allPaths
}

var index = 0

func getPathLengthFromFormat(path string) float64 {
	parts := strings.Split(path, " = ")
	if len(parts) < 2 {
		return math.Inf(1)
	}

	lengthStr := strings.TrimSpace(parts[len(parts)-1])
	length, err := strconv.ParseFloat(lengthStr, 64)
	if err != nil {
		return math.Inf(1)
	}

	return length
}

func findAllPaths(graph Graph, current, end string, visited map[string]bool, currentPath []string, allPaths *[]string, minLength *float64) {
	visited[current] = true
	currentPath = append(currentPath, current)

	if current == end {
		*allPaths = append(*allPaths, formatPath(currentPath, graph))
		if length := calculatePathLength(currentPath, graph); length < *minLength {
			*minLength = length
		}
	} else {
		for _, edge := range graph.Edges {
			if edge.Node1 == current {
				neighbor := edge.Node2
				if !visited[neighbor] {
					findAllPaths(graph, neighbor, end, visited, currentPath, allPaths, minLength)
				}
			} else if edge.Node2 == current {
				neighbor := edge.Node1
				if !visited[neighbor] {
					findAllPaths(graph, neighbor, end, visited, currentPath, allPaths, minLength)
				}
			}
		}
	}

	delete(visited, current)
	currentPath = currentPath[:len(currentPath)-1]

}

func formatPath(path []string, graph Graph) string {
	//formattedPath := strings.Join(path, " -> ")
	length := calculatePathLength(path, graph)

	result := fmt.Sprintf("CS%d: ", index+1)
	for i := 0; i < len(path)-1; i++ {
		node1 := path[i]
		node2 := path[i+1]
		for _, edge := range graph.Edges {
			if (edge.Node1 == node1 && edge.Node2 == node2) || (edge.Node1 == node2 && edge.Node2 == node1) {
				result += fmt.Sprintf("%s - %s - ", node1, edge.Id)
				break
			}
		}
	}
	result += fmt.Sprintf("%s <-------> Longitud de CS%d = %f KM\n ", path[len(path)-1], index+1, length)
	//return fmt.Sprintf("%s: %f", formattedPath, length)
	index++
	return result
}

func calculatePathLength(path []string, graph Graph) float64 {
	length := 0.0
	for i := 0; i < len(path)-1; i++ {
		node1 := path[i]
		node2 := path[i+1]
		for _, edge := range graph.Edges {
			if (edge.Node1 == node1 && edge.Node2 == node2) || (edge.Node1 == node2 && edge.Node2 == node1) {
				weighted, err := strconv.ParseFloat(edge.Weighted, 64)
				if err == nil {
					length += weighted
				}
				break
			}
		}
	}
	return length
}

func distanceMatrix(graph Graph) [][]float64 {
	numNodes := len(graph.Nodes)
	matrix := initializeMatrix(numNodes)
	setDiagonalZeros(matrix)
	fillMatrixWithEdges(matrix, graph.Edges, graph.Nodes)
	calculateShortestDistances(matrix)
	return matrix
}

func formatMatrixAsHTMLTable(matrix [][]float64, nodes Nodes) string {
	var sb strings.Builder

	sb.WriteString("<style>table, th, td {\n  border: 1px solid;\n} td, th {\npadding: 2px;\n}</style><table style='border-collapse: collapse;'>")
	sb.WriteString("<tr><th></th>") // Encabezado de la tabla (nombres de los nodos)
	for _, node := range nodes {
		sb.WriteString("<th>")
		sb.WriteString(node)
		sb.WriteString("</th>")
	}
	sb.WriteString("</tr>")
	for i, row := range matrix {
		sb.WriteString("<tr>")
		sb.WriteString("<th>")
		sb.WriteString(nodes[i])
		sb.WriteString("</th>")
		for j, value := range row {
			cell := fmt.Sprintf("%.2f", value)
			if i == j {
				sb.WriteString("<td style='background-color: yellow;'>")
			} else {
				sb.WriteString("<td>")
			}
			sb.WriteString(cell)
			sb.WriteString("</td>")
		}
		sb.WriteString("</tr>")
	}
	sb.WriteString("</table>")

	return sb.String()
}

func initializeMatrix(size int) [][]float64 {
	matrix := make([][]float64, size)
	for i := 0; i < size; i++ {
		matrix[i] = make([]float64, size)
		for j := 0; j < size; j++ {
			matrix[i][j] = math.Inf(1)
		}
	}
	return matrix
}

func setDiagonalZeros(matrix [][]float64) {
	for i := 0; i < len(matrix); i++ {
		matrix[i][i] = 0
	}
}

func fillMatrixWithEdges(matrix [][]float64, edges Edges, nodes Nodes) {
	nodeIndex := make(map[string]int)
	for i, node := range nodes {
		nodeIndex[node] = i
	}

	// Inicializar todas las distancias con Infinito
	for i := 0; i < len(matrix); i++ {
		for j := 0; j < len(matrix); j++ {
			matrix[i][j] = math.Inf(1)
		}
	}

	// Asignar las distancias entre nodos vecinos
	for _, edge := range edges {
		node1 := nodeIndex[edge.Node1]
		node2 := nodeIndex[edge.Node2]
		weighted, err := parseFloatWeighted(edge)
		if err != nil {
			// Manejar el error si la conversión falla
			logrus.Fatal(err)
			// Puedes optar por omitir esta arista o asignar un valor predeterminado
			continue
		}
		matrix[node1][node2] = weighted
		matrix[node2][node1] = weighted
	}

	// Calcular las distancias mínimas utilizando el algoritmo de Dijkstra
	for i := 0; i < len(matrix); i++ {
		dijkstra(matrix, i)
	}
}

func dijkstra(matrix [][]float64, start int) {
	size := len(matrix)
	visited := make([]bool, size)
	distances := make([]float64, size)

	// Inicializar todas las distancias con Infinito excepto el nodo de inicio
	for i := 0; i < size; i++ {
		distances[i] = math.Inf(1)
	}
	distances[start] = 0

	// Encontrar el nodo con la distancia mínima en cada iteración
	for count := 0; count < size-1; count++ {
		u := minDistance(distances, visited)
		visited[u] = true

		// Actualizar las distancias de los nodos adyacentes
		for v := 0; v < size; v++ {
			if !visited[v] && matrix[u][v] != math.Inf(1) && distances[u]+matrix[u][v] < distances[v] {
				distances[v] = distances[u] + matrix[u][v]
			}
		}
	}

	// Asignar las distancias mínimas a la matriz de distancia
	for i := 0; i < size; i++ {
		matrix[start][i] = distances[i]
		matrix[i][start] = distances[i]
	}
}

func minDistance(distances []float64, visited []bool) int {
	min := math.Inf(1)
	minIndex := -1
	for i, distance := range distances {
		if !visited[i] && distance < min {
			min = distance
			minIndex = i
		}
	}
	return minIndex
}

func calculateShortestDistances(matrix [][]float64) {
	size := len(matrix)
	for k := 0; k < size; k++ {
		for i := 0; i < size; i++ {
			for j := 0; j < size; j++ {
				if matrix[i][j] > matrix[i][k]+matrix[k][j] {
					matrix[i][j] = matrix[i][k] + matrix[k][j]
				}
			}
		}
	}
}

//func distanceMatrix(graph Graph) [][]float64 {
//	// Crear una matriz cuadrada para almacenar las distancias entre nodos
//	numNodes := len(graph.Nodes)
//	matrix := make([][]float64, numNodes)
//	for i := 0; i < numNodes; i++ {
//		matrix[i] = make([]float64, numNodes)
//	}
//
//	// Inicializar la matriz con infinito para todas las distancias
//	for i := 0; i < numNodes; i++ {
//		for j := 0; j < numNodes; j++ {
//			matrix[i][j] = math.Inf(1)
//		}
//	}
//
//	// Establecer la distancia cero para los nodos diagonales
//	for i := 0; i < numNodes; i++ {
//		matrix[i][i] = 0
//	}
//
//	// Llenar la matriz con las distancias de las aristas existentes
//	for _, edge := range graph.Edges {
//		index1 := getNodeIndex(graph.Nodes, edge.Node1)
//		index2 := getNodeIndex(graph.Nodes, edge.Node2)
//		matrix[index1][index2] = 1
//		matrix[index2][index1] = 1
//	}
//
//	// Calcular las distancias mínimas utilizando el algoritmo de Floyd-Warshall
//	for k := 0; k < numNodes; k++ {
//		for i := 0; i < numNodes; i++ {
//			for j := 0; j < numNodes; j++ {
//				if matrix[i][j] > matrix[i][k]+matrix[k][j] {
//					matrix[i][j] = matrix[i][k] + matrix[k][j]
//				}
//			}
//		}
//	}
//
//	return matrix
//}

// Función auxiliar para obtener el índice de un nodo en el slice de nodos
func getNodeIndex(nodes Nodes, node string) int {
	for i, n := range nodes {
		if n == node {
			return i
		}
	}
	return -1
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
