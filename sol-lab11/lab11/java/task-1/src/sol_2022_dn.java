import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Scanner;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.PriorityQueue;

public class sol_2022_dn {
    static class Task {
        public static final String INPUT_FILE = "in";
        public static final String OUTPUT_FILE = "out";

        // numarul maxim de noduri
        public static final int NMAX = 200005;

        // valoare mai mare decat orice distanta din graf
        public static final int INF = (1 << 30);

        // n = numar de noduri, m = numar de muchii
        int n, m;

        // adj[node] = lista de adiacenta a nodului node
        // Edge e inseamna ca exista muchie de la e.node la e.neigh de cost e.w
        ArrayList<Edge> edges = new ArrayList<>();

        public class Pair {
            public int x;
            public int y;

            Pair(int _x, int _y) {
                x = _x;
                y = _y;
            }
        }

        public class Edge {
            int node;
            int neigh;
            int w;

            Edge(int _node, int _neigh, int _w) {
                node = _node;
                neigh = _neigh;
                w = _w;
            }
        };

        // structura folosita pentru a stoca MST
        class MSTResult {
            int cost; // costul MST-ului gasit

            ArrayList<Pair> edges; // muchiile din MST-ul gasit (ordinea nu conteaza)

            MSTResult(int _cost,  ArrayList<Pair> _edges) {
                cost = _cost;
                edges = _edges;
            }
        };

        // Structura de date descrisa aici https://infoarena.ro/problema/disjoint.
        public class DisjointSet {
            // parent[node] = radacina arborelui din care face parte node.
            // (adica identificatorul componentei conexe curente)
            int [] parent;

            // size[node] = numarul de noduri din arborele in care se afla node acum.
            int [] size;

            // Se initializeaza n paduri.
            DisjointSet(int nodes) {
                parent = new int[nodes + 1];
                size   = new int[nodes + 1];
                // Fiecare padure contine un nod initial.
                for (int node = 1; node <= nodes; ++node) {
                    parent[node] = node;
                    size[node] = 1;
                }
            }

            // Returneaza radacina arborelui din care face parte node.
            int setOf(int node) {
                // Daca node este radacina, atunci am gasit raspunsul.
                if (node == parent[node]) {
                    return node;
                }

                // Altfel, urcam in sus din "radacina in radacina",
                // actualizand pe parcurs radacinile pentru nodurile atinse.
                parent[node] = setOf(parent[node]);
                return parent[node];
            }

            // Reuneste arborii lui x si y intr-un singur arbore,
            // folosind euristica de reuniune a drumurilor dupa rank.
            void union(int x, int y) {
                // Obtinem radacinile celor 2 arbori
                int rx = setOf(x), ry = setOf(y);

                // Arborele mai mic este atasat la radacina arborelui mai mare.
                if (size[rx] <= size[ry]) {
                    size[ry] += size[rx];
                    parent[rx] = ry;
                } else {
                    size[rx] += size[ry];
                    parent[ry] = rx;
                }
            }
        }

        public void solve() {
            readInput();
            writeOutput(getResult());
        }

        private void readInput() {
            try {
                Scanner sc = new Scanner(new BufferedReader(new FileReader(
                                INPUT_FILE)));
                n = sc.nextInt();
                m = sc.nextInt();

                for (int i = 1; i <= m; i++) {
                    int x, y, w;
                    x = sc.nextInt();
                    y = sc.nextInt();
                    w = sc.nextInt();
                    edges.add(new Edge(x, y, w));
                }
                sc.close();
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }

        private void writeOutput(MSTResult result) {
            try {
                PrintWriter pw = new PrintWriter(new File(OUTPUT_FILE));
                pw.printf("%d\n", result.cost);
                for (Pair e : result.edges) {
                    pw.printf("%d %d\n", e.x, e.y);
                }
                pw.close();
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }

        private MSTResult getResult() {
            //
            // TODO: Calculati costul minim al unui MST folosind Kruskal.
            //
            //
            // Vi se da implementarea DisjointSet. Exemple utilizare:
            //      DisjointSet disjointset = new DisjointSet(n);
            //      int setX = disjointset.setOf(x);
            //      ...
            //      disjointset.union(x, y);
            //

            return kruskal(n, edges);
            // return prim(n, edges, 1);
        }

        // MST generat cu Kruskal.
        MSTResult kruskal(int nodes, ArrayList<Edge> edges) {
            // Sortam muchiile crescator dupa cost.
            Collections.sort(edges, new Comparator<Edge>() {
                public int compare(Edge a, Edge b) {
                        return Integer.compare(a.w, b.w);
                };
            });

            // Initializam padurile.
            DisjointSet disjointset = new DisjointSet(nodes);

            // Initializam MST: cost 0, fara muchii
            int cost = 0;
            ArrayList<Pair> mst = new ArrayList<>();

            // Folosim muchiile in ordine crescatoare a costului.
            int used_edges = 0;
            for (Edge edge : edges) {
                int x = edge.node, y = edge.neigh, w = edge.w;

                // Aflam radacinile celor 2 arbori in care sunt x si y.
                if (disjointset.setOf(x) !=  disjointset.setOf(y)) {
                    // Reunim arborii.
                    disjointset.union(x, y);

                    // Adaugam muchia la MST.
                    cost += w;
                    mst.add(new Pair(x, y));

                    ++used_edges;
                    if (used_edges == nodes - 1) {
                        // Daca am format deja un arbore, ne putem opri.
                        break;
                    }
                }
            }

            return new MSTResult(cost, mst);
        }

        public class PairPrim implements Comparable<PairPrim> {
            public int destination;
            public int cost;

            PairPrim(int _destination, int _cost) {
                destination = _destination;
                cost = _cost;
            }

            public int compareTo(PairPrim rhs) {
                return Integer.compare(cost, rhs.cost);
            }
        }

        // MST generat cu Prim. Alegem pentru simplitate nodul de start 1.
        private MSTResult prim(int nodes, ArrayList<Edge> edges, int source) {
            // adj[node] = lista de adiacenta a lui node: (neigh, w)
            ArrayList<PairPrim> adj[] = new ArrayList[NMAX];
            for (int node = 1; node <= nodes; ++node) {
                adj[node] = new ArrayList<>();
            }
            for (Edge e : edges) {
                adj[e.node].add(new PairPrim(e.neigh, e.w));
                adj[e.neigh].add(new PairPrim(e.node, e.w));
            }

            // d[node] = distanta nodului node fata de MST-ul curent (cel ma apropiat nod din MST)
            // d[node] in aceasta problema va fi mereu egal cu costul unei muchii
            // Initializam vectorul de distante cu distante infinite.
            int[] d = new int[n + 1];
            // p[node] = parintele lui node (initializat cu 0)
            int[] p = new int[n + 1];
            boolean[] used = new boolean[n + 1];

            // Initializam distantele la infinit.
            for (int node = 0; node <= nodes; node++) {
                d[node] = INF;
                p[node] = 0;
            }

            // Folosim un priority queue de PairPrim, desi elementele noastre nu sunt tocmai muchii.
            // Vom folosi field-ul de cost ca si distanta pana la nodul respectiv.
            // Observati ca am modificat clasa Edge ca sa implementeze interfata Comparable.
            PriorityQueue<PairPrim> pq = new PriorityQueue<>();

            // Inseram nodul de plecare in coada si ii setam distanta pe 0.
            d[source] = 0;
            pq.add(new PairPrim(source, 0));

            // Initializam MST: cost 0, fara muchii
            int cost = 0;
            ArrayList<Pair> mst = new ArrayList<>();

            // Cat timp mai sunt noduri in coada
            while (!pq.isEmpty()) {
                // Scoatem un nod din coada
                int node = pq.poll().destination;
                // In cazul in care nodul e deja in MST, ignoram aceasta intrare.
                if (used[node]) {
                    continue;
                }

                // Adaug muchia node - p[node].
                used[node] = true;
                // Nodul radacina este adaugat print-o muchie fictiva,
                // care nu face parte din MST.
                if (p[node] != 0) {
                    cost += d[node];
                    mst.add(new Pair(node, p[node]));
                }

                // Ii parcurgem toti vecinii.
                for (PairPrim e : adj[node]) {
                    int neigh = e.destination, w = e.cost;

                    // Se imbunatateste distanta?
                    if (!used[neigh] && w < d[neigh]) {
                        // Actualizam distanta si inseram din nou in pq.
                        d[neigh] = w;
                        p[neigh] = node;
                        pq.add(new PairPrim(neigh, d[neigh]));
                    }
                }
            }

            return new MSTResult(cost, mst);
        }
    }

    public static void main(String[] args) {
        new Task().solve();
    }
}
