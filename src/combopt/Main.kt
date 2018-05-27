package combopt

import gurobi.*
import java.io.File
import java.nio.charset.StandardCharsets
import java.util.*
import kotlin.math.abs

fun main(args: Array<String>) {
    if (args[0].isBlank() || args[1].isEmpty()) {
        throw IllegalArgumentException("Either argument 1 or 2 is empty")
    }

    val inputFile = args[0]
    val distances = calculateDistances(readFile(inputFile))
    val result = solveTsp(distances)

    saveToFile(args[1], result)
}

fun checkDistances(distances: Array<Array<Array<Pixel>>>) {
    for (i in 0 until distances.size - 1) {
        for (j in i + 1 until distances.size) {
            println("Distance between image ${i} and ${j} " +
                    "is ${calculateDistance(distances[i], distances[j])}, " +
                    "reverse is ${calculateDistance(distances[j], distances[i])}")
        }
    }
}

fun solveTsp(distances: Array<Array<Int>>): List<Int> {
    val env = GRBEnv()
    val model = GRBModel(env)
    val nodes = distances.size

    val x = Array(nodes, {
        Array(nodes, {
            model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "x_$it")
        })
    })
    model.update()

    val objective = GRBLinExpr()
    setObjective(objective, x, distances)
    model.setObjective(objective, GRB.MINIMIZE)

    addEnterOnceCons(model, x)
    addLeaveOnceCons(model, x)
    addStartNodeCons(model, x)
    addTargetNodeCons(model, x)

    model.set(GRB.IntParam.LazyConstraints, 1)
    model.setCallback(SubroutesCallback(x))
    model.optimize()

    val objValue = model.get(GRB.DoubleAttr.ObjVal)
    println("Obj value: $objValue")

    val path = LinkedList<Int>()
    var currentNode = 0
    do {
        path.add(currentNode)
        for (i in x.indices) {
            if (x[currentNode][i].get(GRB.DoubleAttr.X) > 0) {
                currentNode = i
                break
            }
        }
    } while (currentNode != 0)

    return path.subList(1, path.size)

}

fun addStartNodeCons(model: GRBModel, x: Array<Array<GRBVar>>) {
    val cons = GRBLinExpr()
    for (j in 0 until x.size) {
        cons.addTerm(1.0, x[0][j])
    }
    model.addConstr(cons, GRB.EQUAL, 1.0, "start_node")
}

fun addTargetNodeCons(model: GRBModel, x: Array<Array<GRBVar>>) {
    val cons = GRBLinExpr()
    for (i in 0 until x.size) {
        cons.addTerm(1.0, x[i][0])
    }
    model.addConstr(cons, GRB.EQUAL, 1.0, "end_node")
}

class SubroutesCallback(private val x: Array<Array<GRBVar>>) : GRBCallback() {
    override fun callback() {
        if (where == GRB.CB_MIPSOL) {
            val path = LinkedList<Pair<Int, Int>>()
            val visited = Array(x.size, { false })
            var currentNode = 0

            while (!visited[currentNode]) {
                visited[currentNode] = true
                for (i in x.indices) {
                    if (getSolution(x[currentNode][i]) > 0) {
                        path.add(Pair(currentNode, i))
                        currentNode = i
                        break
                    }
                }
            }

            if (path.size != x.size) {
                val cons = GRBLinExpr()
                for (edge in path) {
                    cons.addTerm(1.0, x[edge.first][edge.second])
                }
                addLazy(cons, GRB.LESS_EQUAL, (path.size - 1).toDouble())
            }
        }
    }
}

fun addLeaveOnceCons(model: GRBModel, x: Array<Array<GRBVar>>) {
    for (i in x.indices) {
        val cons = GRBLinExpr()
        for (j in x.indices) {
            cons.addTerm(1.0, x[i][j])
        }
        model.addConstr(cons, GRB.EQUAL, 1.0, "leave_once_node_$i")
    }
}

fun addEnterOnceCons(model: GRBModel, x: Array<Array<GRBVar>>) {
    for (j in x.indices) {
        val cons = GRBLinExpr()
        for (i in x.indices) {
            cons.addTerm(1.0, x[i][j])
        }
        model.addConstr(cons, GRB.EQUAL, 1.0, "enter_once_node_$j")
    }
}

fun setObjective(objective: GRBLinExpr, x: Array<Array<GRBVar>>, distances: Array<Array<Int>>) {
    x.forEachIndexed { i, xArr ->
        xArr.forEachIndexed { j, grbVar ->
            objective.addTerm(distances[i][j].toDouble(), grbVar)
        }
    }
}

fun calculateDistances(distances: Array<Array<Array<Pixel>>>): Array<Array<Int>> {
    val nodes = distances.size
    val dist = Array(nodes + 1, { Array(nodes + 1, { Int.MAX_VALUE }) })
    for (i in 0 until dist.size - 1) {
        for (j in i + 1 until dist.size) {
            if (i == 0 || j == 0) {
                dist[i][j] = 0
                dist[j][i] = 0
            } else {
                dist[i][j] = calculateDistance(distances[i - 1], distances[j - 1])
                dist[j][i] = calculateDistance(distances[j - 1], distances[i - 1])
            }
        }
    }
    return dist
}

fun calculateDistance(img1: Array<Array<Pixel>>, img2: Array<Array<Pixel>>): Int {
    val widthIdx = img1[0].size - 1
    var distance = 0
    for (height in img1.indices) {
        for (color in 0..2) {
            distance += abs(img1[height][widthIdx].colors[color] - img2[height][0].colors[color])
        }
    }
    return distance
}

fun saveToFile(path: String, result: List<Int>) {
    File(path).writeText(result.joinToString(" "))
}

fun readFile(inputFile: String): Array<Array<Array<Pixel>>> {
    val reader = File(inputFile).bufferedReader(StandardCharsets.UTF_8)
    val parameters = reader.readLine().split(' ')
    val (nodes, width, height) = parameters
    val distances: Array<Array<Array<Pixel?>>> = Array(nodes.toInt(), {
        Array(height.toInt(), {
            arrayOfNulls<Pixel>(width.toInt())
        })
    })
    reader.useLines { sequence ->
        sequence.forEachIndexed({ nodeNumber, line ->
            val nodePixels = line.split(' ')
            for (heightIdx in distances[nodeNumber].indices) {
                for (widthIdx in distances[nodeNumber][heightIdx].indices) {
                    val cursor = heightIdx * width.toInt() * 3 + widthIdx * 3
                    distances[nodeNumber][heightIdx][widthIdx] =
                            Pixel(nodePixels[cursor].toInt(), nodePixels[cursor + 1].toInt(), nodePixels[cursor + 2].toInt())
                }
            }
        })
    }
    return distances as Array<Array<Array<Pixel>>>
}