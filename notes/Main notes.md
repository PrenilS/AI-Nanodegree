#AIND notes
##1: Intro to AI
Six main diciplines:
- NLP
- Knowledge representation
- Automated reasoning
- Machine learning
- Computer vision
- Robotics

Four different sort of goals:
- Think and act humanly
- Think and act rationally

Notes:
Heuristic is additional info/rules/constraints which inform what would otherwise be a brute-force algorithm.
A* search algorithm checks all options and prioritise those that minimise the distance.

Need to keep all neccecary info to succeed.

Adversarial Search algorithms:
Mini-max algorithm (max chances of you winning and min chances of oponent winning)

Bayes Theory to solve Month Hall problem P(a|b)=P(b|a)*P(a)/p(b)


States (info) to store:
Only things that are necessary to know to complete the task.
Goal state:
States that represent the goal you want to achieve.

Preception:
Sensing properties of environment

Actions:
Change state of the environment

Cognition:
Decising what actions to take based on percieved inputs.

Intelligent agent: takes actions to max expected utility given a goal (rational behavior).
	Cant expect to always operate optimally in real life. Instead, use bounded optimality.

Reactive/Behaviour based agents: Actions and Perception are directly links


Environment:
Fully/Partially observable
Determinstic/Stochastic
Discrete/Continous
Benign/Adversarial




2,6.1-6.5,3

##Constraint propagation & Search

Constraint propagation narrows possibilities and means that you need to search less.
Search to find all possibilities at each step and keep track of them.

Search tries a number of possible solutions when it is not clear which solution is the correct one to use.
It branches solutions out until it finds one that works.

Depth first search:
In tree of possibilities, start at root and move over all nodes in the left most branch then move to the next branch on the right.

###Constraint satisfaction problems (CSP):

Variables, domains, constraints

Constraint graphs:
Nodes are variables and lines are constraints.

Search in csp's: assign guesses to variable values to get to next state untill all have a valid value. If not then go back and repeat.
	Depth first search will get all possibilities but will test some that dont make sense and also test the same option more than once.
	Backtracking is the same as depth first search but looks at one assignment order and reverts to a different one
		once any of its constraints are violated.
		Choice of which var to choose first and what value to use affects efficiency.
	Local searches use complete assignments. All variables always are given a value. Start with full assignment then 
	change one or more variables in a local neighbourhood.
Can improve efficiency:
	Fist assign those with min remaining values (mrv).
	First assin those with most constraints.
	Forward checking keeps list of possible values left for each variable. When a variable ends up with no remaining values
	then can backtrack sooner.
		Networks are arc consistent is for each possibility in a given node, there are other viable possibilies
		which can be taken in the nodes connected to it. Network is consistent if all arc are arc consistent.

Monolithic constraints vs splitting constraints up.
All boolean constraints vs pseudo-boolean cardinality constraint.

Defintion of a problem:
-Initial state: All states are the state space and navigat the space with actions.
-Actions: state dependent or not
-Result: of a state and an action->new state R(s,a)->s'
-Goal test: to see if you have the result or not
-Path cost function: cost of states and actions over all steps. Probaly additive
	-Sum of step cost functions cost(s,a,s')

###Uninformed Search
-No extra info about states appart from what is provided in the problem
-Just have to keep generating successors until they get to the goal state.
-States can be put into 3 types. Explored, unexplored and frontier which is the furthest
	state you have reached in each link.

####Tree search:
-Look at frontier. If not more options there then fail
-If not the remove that option from the frontier
	-different algos choose how to eliminate the frontier options differently
-if it is the goal then you are done
-if not goal then look at actions avaliable.

####Breath first search (closest first):
-Expands shallowest (shortest) paths first
-Chooses path from frontier which has not been chosen yet and is the 
	shortest (in steps)
-When all same distance then choose at random or have a different kind of tie breaker
-can easily end up backtracking.
-Graph search- dont go back to states already in explored
- Wont just stop when goal is found. Can add constraint to do that.
-Doesnt know if found best path or not. Looks so far at only steps and not cost.
-Frontier size is 2^n at level n
-This search is complete. Will find goal even if infinite length paths

####Uniform cost search (cheapest first):
-Expands cheapest cost path first
-doesnt expand by looking for nearest but cheapest path.
-Add cost on for successive steps and new paths
-cant go back to node already explored. Will be more expensive anyway.
-Continue to search until goal is taken off the frontier (until you explore it)
-always finds cheapest as long as all costs are non negative.
-Frontier size is 2^n at level n
-This search is complete. Will find goal even if infinite length paths
-starts from startstate and expands equally in all directions.
-Will spend lots of extra effort to reach goal.

####Depth first search:
-Expands first the longest path
-terminates the first time it finds the goal. Not optimal like the other two.
-Frontier size is just n as long as dont keep track of explored states.
- Not complete. Wont be able to find goal in infinite length paths. Can just
	get stuck in one inifinite path which doesnt include goal.

###Informed search

--additional information about the search states

####Greedy best-first search:

-Search directed towards the goal by estimates of how close you get to the goal
-Finds a path. Expands nodes in one direction only. But can end up taking a
a longer route.
-If a direct path is available, expends much less effort than Uniform Cost
- Better if can combine using fewer nodes but still find the shortest path.

####A* algorithm (best estimated total path cost first):

-expands paths with min value of f=g+h
-g(path)=path cost to get to next state, 
	h(path)=h(s)=estimated distance to the goal from next state you want to go to, 
s is state you are in
-So its the cost to get to where you are+estimated cost to get to goal from here
-min(g) keeps path short, min(h) keeps focused on the goal,
min(f) finds shortest length path to the goal which expanding the minimum
number of paths possible.
-Doesnt just focus on one path. Still considers every path on frontier
when deciding what has the smallest f value.
-Will generally find the lowest cost function but depends on having the 
correct h function to do so. This happens when h is optimistic/admissible.
-Finds lowest cost path when h(s)<true cost to get to goal from s.
h should not overestimate the distance to the goal.
This is because once path to goal is found, it is lower in cost than any other
on the frontier and we know its true cost since h=0 at goal. Also everything on
frontier will be underestimating their cost to the goal and we are already
lower than that anyway. Also all steps are thought to have non zero cost.
So cost should keep going up for other paths.

--An admissible heuristic never over estimates the cost to get to the goal. 
An A* search with one heuristic that is more costly (>=) than another 
but still admissible will expand fewer paths to get to the goal.
--A consistent heuristic will never decrease cost by more than one for an action
which has a cost of 1.
--Heuristics can be combined so that new=max(old1,old2). Will be admissible as
long as both old ones are admissible and will expand fewer paths.
Disadvantage is the extra cost for computing two.

--Paths<->Nodes. Nodes have state they represent, action taken to get there,
cost to get there and the parent node.
--Nodes fall into frontier or explored lists.
Frontier
-Removes best items and adds in new ones and tests membership. 
It is implemented as a priority queue which knows what to add when. 
Represented as a set and built with a hast table or tree.
Explored
-Adds new members tests membership to set. Represented as a single set 
and built from a hast table or tree.


Problem solving will always work when:
-Domain needs to be fully observable: Should be able to see initial state
-Domain needs to be known: Should know all actions avaliable to us
-Domain needs to be dicrete: Should be a finite number of actions to choose from
-Domain needs to be deterministic: Should know what the result of any action will be
-Domain needs to be static: Nothing else can change the world apparty from agents actions

##Propositional logic:

Prop logic sentence is true or falls wrt a model of the world.
Model is true/false values for the world:{E:True, B:False...}

Truth tables:
Show the possible value of each variable (true/false usually) 
in the model and how the relate to each other by showing the values
of different sentences involving them.

Implies in prop logic can have things which arent related and still say true
so as long as both are true then true=>true is true.

Example:
	E-earthquake
	B-burglary
	A-alarm
	M/J-M or J calling
(E v B)>A so E OR B is true implies A is true
A>(J^M) A is true implies that M AND J both call
J<>M for if and only if
J<>-,M if one then not the other
E=>A is T when both are T or F and when E is F and A is T
it is only F when E is T and A is F.

Valid sentence:
True in every possible model

Satisfiable sentence:
True in some models

Limitations:
-Can only handle T/F not uncertainty
-Cant talk about objects with properties or their relations
-No shortcuts there is no all to say 100 things are T needs 100 conjunctions.

=======================================================================
#Ontological commitment: What they say about the world
*Epistemological commitment: Beliefs that agent can have


First order logic
#Relations, objects, functions
*T/F/?

Propositional logic
#Facts
*T/F/?

Probability theory
#Facts
*[0,1]

Problem solving is atomic, its broken down into bits or states and we know certain things about those states

Prop logic and Prob theory are AB factored since representation of the world is factored into certain variables

First ordered logic is structured. It has the variables but also relationships between them.

##First Order Logic:
Model:
-Objects
-Constants refering to objecs but doesnt have to have 1-to-1 correspondence 
-Functions
-Relations

Syntax:
-Sentences: Describe facts
	Predicates corresponding to relations so vowel(A) and equality is in every model 
	Can be combined with operators and ^, or V, not -. , implies =>, equivalent <=> and brackets
	Quantifiers: All and there exists

-Terms: Describe objects
	Constants
	Variables
	Functions
Planning:

Problem solving vs planning:
Planning alone wont always work when executing since properties of environment may change. So any time env is not deterministic.
So issues are for:
-Stochastic envs
-Multiagent scenarios
-Partial observability (dont always know exactly which state we start in)


Solve by planning for belief states (number of possible states. Once action is done then you get more info and change belife state)
 and not objective world states.

Can also deal with different kinds of envs:
Deterministic and fully observable
Deterministic and partially observable
Stochastic

For deterministic you start then have an action then get a result then move again and get new result until you are in a goal state
For stochastic you start, predict state from belief states then apply and action and update your prediction based on the action.

For stochasic and partially observable you might need and inifite number of plans to be sure that the goal is achieved.
Can impliment this by doing action-> observe result then loop back and repeat or carry on. Can be done with a more complicated seearch tree.

Bounded Solution:
-Can find the solution in a bounded number of steps as long as there are no loops in the tree

Unbounded Solution:
-Can find the solution in an unbounded number of steps as long as every leaf node is a goal.



============================================================================================================================================

##Classical planning:

State space: k-booleans =2^k variables
World state: complete assignment of true and false to each variable
Belief state: Complete assignment (deterministic, fully observable
	Partial assignment
	Arbitary formula
Action schema: Set of all possible actions in the schema, what need to know to apply the action and what the effect of the action will be.
	Will also come with an initial state and goal state.

Planning can be done same as problem solving with a progression search:
Go from one state to another by branching over different actions until you reach a goal. This a forward or state space search.
But with planning representation, can do more than this.

Regression (backwards) search:
Start at goal state. Goal state represents a family of things that can result in that state.
Look at action schema to figure out which actions result in that goal to work backwards.

Regression vs Progression:
Regression can result in far fewer nodes expanded than in a forward search.


**Fluent:** Condition that changes over time
**Ground term::** Does not contain any free variables

World is represented using a factored representation (a collection of variables).

**PDDL (Planning domain definition language)** used for the action schema. This allows all actions to be expressed in
one action schema. 
Describes all 4 things needed to define a search problem:
1. Initial state

Each state is represented as a conjunction of fluents that are ground, functionless atoms. Closed world assumption
so any fluents not mentioned are assumed to be false. State can be seen as a conjunction of fluents for logical
inference or a set of fluents for set operations.

2. Actions available in that state

Set of action schemas which are needed to define the actions(s) and results(s,a) functions in a search.
Set of ground actions is represented in a single **action schema** (action name, vars, preconditions and effects). 

**Entailment:** s |= q iff every positive literal in q is in s and every negated literal in q is not.
(a is element of Actions(s))<=> s|= Precond(a)

An action is applicable in state s if its preconditions are satisfied by s.

3. Result of applying the action

Result of action a in state s is s'= set of fluents starting with s and deleting those that are negatives in the 
effects and adding those that are positive. (delete list DEL(a) and add list ADD(a)).
Fluents here don't refer to time at all since implicit in action schema.

RESULT(s,a)=(s-DEL(a)) U ADD(a)

4. A goal test

Initial state is conjunction of ground atoms.
Goal is conjunction of literals and variables where a variable means any value of that type will satisfy it.


### Complexity in classical planning

PlanSAT: if there is a solution to solve the planning problem. Decidable as long as no function symbols. Sub-optimal plan

Bounded PlanSAT: if there is a solution of length k or less. Decidable even with function symbols. Optimal plan.

Both are in complexity class PSPACE > NP. Without negative preconditions, PlanSAT reduces to complexity P.

### Algorithms

Can be solved with heuristic or local searches.

**Forward state space search:**

Very slow for large problems. Domain independent heuristics which can be derived automatically must be used to make 
feasible. Any applicable action cant be used to extend the search tree.

**Backward relevant-states search**

Start with goal and search backwards. n ground fluents= 2^n ground states but 3^n sets of goal states.
PDDL allows any domain that is explained in it to be regressed on.

For ground description g and ground action a. Regressing g over a gives state g'=(a-ADD(a)) U Precond(a).

For best results must also use partially uninstantiated actions and states, not just ground ones.

Only relevant actions can be used to extend the search tree. Action must have at least one effect that unifies with an
element of the goal and must not have any effects which negate an element of the goal.

Fewer branches but harder to form usefull heuristics.

**Planning heuristics**

Heuristic estimates distance from state to goal. A* search can be used as long as you have an admissible heuristic
(doesn't overestimate the distance). Can derived by defining a more relaxed problem that is easier to solve.
Can add more edges to graph or group nodes together.

Atomic states need domain-specific heuristics.

Planning with states and action schemas allows use of domain-independent heuristics.

**Ignore preconditions heuristic**

Adds edges to graphs by dropping all preconditions from actions. So can apply any action to any state.
Ignore actions that negate any goals and consider those that do at least one unsatisfied goal.
Count min number of actions to get to the goal. Instance of set-cover problem (NP hard) and can be solved in 
log n if a greedy alg is used but then the heuristic is not admissible.

Can ignore only some preconditions instead of all.

**Ignore or delete lists heuristic**

Remove delete lists from the actions so you progress towards goal always (assuming no negatives in teh problem).
Still NP-hard. Can be solved in polynomial time by hill-climbing.

**State abstraction**

Since both are still NP-hard, need to reduce states. Can try to drop certain fluents.

In heuristics can use decomposition to divide a problem into parts.
Can solve each part independently, and then combine the parts. 

The **subgoal independence assumption** is that the cost of solving a conjunction of subgoals is approximated 
by the sum of the costs of solving each subgoal independently. Optimistic is negative interactions between subplans
for each goal and pessimistic (inadmissible) when subplans contain redundant actions. Only when subgoals are
independent, then summing is admissible. If not have to use max cost of the individual plans for each of the goals.

###Planning graphs

Can give better heuristics and can be solved by GRAPHPLAN.

Tree of all actions and possible states can be searched to find a solution. Tree will have exponential size.
Planning graph is an approximation of this and has only polynomial size. It doesnt give a definite answer like a tree 
but can estimate how many steps needed to get to the goal.

Always correct when answer is that the goal is not reachable. Also never overestimates the number of steps.
Hence, it is **admissible**.

A planning graph is a directed graph organized into levels: 

First is the initial state S0. Nodes represent each fluent that holds in that state
 
Next is level A0, Nodes are each ground action applicable in S0;
 
Levels Si (all literals that could hold at time i) and Ai (all actions that could have preconditions met at time i)
for i until termination come after.  Each action at level Ai is connected to its preconditions at Si and its effects 
at Si+1. 

**Persistence actions:** preserves the fluents truth

So literals appear when actions cause them but when they nothing negates it then it PERSISTS. This is
represented by persistence actions (precondition C and effect C).

**Mutex actions:** When one action negates the effect of another OR the effect of one action
negates the preconditions of another OR
the preconditions for one effect are mutually exclusive to those of the other effect

Lines also indicate mutual exclusion (or mutex) links. 
There are literals that could not appear together,
regardless of the choice of actions. 

Si represents a set of possible states. Each possible state is a
subset of the literals with no mutex links.

**Mutex literals:** If one is a negation of the other OR each possible pair of actions to achieve both
literals are mutex.

Repeat until two consecutive levels are identical (graph has **Leveled off**.

Planning graphs record only a restricted subset of the possible negative interactions among
actions. The are polynomial size. For l literals and a actions and n levels is O(n(a+l)^2))

Planning graphs work only for propositional planning problems (no variables). 
Action schemas can however be propositionalized. 

### Planning graph heuristics

If any goal literal is not in the final level then unsolvable.

Estimated cost of goal literal gi from s is number of levels from s where gi first shows up. This is the 
**Level cost** of gi. Not accurate since many actions are possible on each level.

So use serial planning graph to calculate heuristics. Add mutex links between every pair of nonpersistence actions.
So only one action can happen at a time. This can be used to get better level costs to estimate the actual.

Then for all the goal literals in conjunction

**Max-level:** max(level costs for gi). Admissible but not great.

**Level sum:** assumes subgoal independence so not always admissible. Good for decomposable problems. Sum of
level costs of all the goals.

**Set level:** Level where all literals in conjunctive goal appear without any pair being mutex. Admissible. Ignores
interactions between more than two literals.

###GRAPHPLAN

    function GRAPHPLAN(problem) returns solution or failure 

        graph ← INITIAL-PLANNING-GRAPH(problem)
    
        goals ← CONJUNCTS(problem.GOAL)
        nogoods ←an empty hash table
    
        for tl = 0 to ∞ do:
    
            if goals all non-mutex in St of graph then
                solution ← EXTRACT-SOLUTION(graph, goals, NUMLEVELS(graph), nogoods)
                if solution = failure then return solution
            if graph and nogoods have both leveled off then return failure
            graph ← EXPAND-GRAPH(graph, problem)

Extract solution can be a Boolean CSP with variables the actions with values in or out of plan and constraints
 the mutex links and goals.
 
 Or can be a backwards search where the initial state is the last level of the planning graph.
 Actions available in a state at level Si are to select any conflict- (no two of them are mutex and no two
of their preconditions are mutex) subset of the
actions in Ai−1 whose effects cover the goals in the state. The resulting state has level
Si−1 and has as its set of goals the preconditions for the selected set of actions.Cost of each action is 1.

When EXTRACT-SOLUTION fails to find a solution for a set of goals at
a given level, we record the (level, goals) pair as a no-good and when EXTRACT-SOLUTION is called again 
with the same level and goals immediately return

In the worst case the backwards search may be intractable. So need a heuristic. Greedy algorithm based on
the level cost of the literals:

1. Pick literal with the highest level cost.
2. To achieve that literal, prefer actions with easier preconditions (sum (or maximum) of the level costs of
 its preconditions is smallest.)
  
GRAPHPLAN will terminate and return failure when there is no solution.

When graph itself and the no-goods have both leveled off, with no solution found, we can terminate with
failure because there is no possibility of a subsequent change that could add a solution.
No-goods level off since

• Literals increase monotonically

• Actions increase monotonically

• Mutexes decrease monotonically

• No-goods decrease monotonically  


Currently the most popular and effective approaches to fully automated planning are:

• Translating to a Boolean satisfiability (SAT) problem

Propositional logic

• Forward state-space search with carefully crafted heuristics

• Search using a planning graph

PDDL cant have universal quantifiers like first-order logic.

*Plan space search*
Searches through space of plans instead of space of states. Starts with start state and goal state and adds
actions to keep branching the possible different plans there could be. Then search over those plans. 

*Backwards search* searches abstract plan states

*Forwards search* searches concrete plan states. Can use good heuristics. Action schemas allow
heuristics to be generated automatically. Just have to relax some of the requirements in the action schema.

For sliding blocks:

```buildoutcfg
Action: Slide(t,a,b)
Precondition: On(t,a) ^ Tile(t) ^ Blank(b) ^ Adj(a,b)
Effect: On(t,b) ^ Blank(a) ^ -,On(t,a) ^ -, Blank(b)
```

If remove precondition that `Blank(b)` to relax the prob then we have manhattan distance. Or could remove
`Adj(a,b)`.

Can also ignore negative effects.

### Situation calculus

ALL doesn't exist in propositional languages like universal quantifiers.
But First order logic can do this.

To apply this first order logic to planning, we use situation calculus.

Actions are seen as objects (in the form of a function Fly(p,c,a))

Situations are seen as objects too corresponding to paths instead of states. Initial situation is S0. 
Then there is a function result of applying an action to the situation `S'=Result(s,a)` to produce a new situation.
Don't say what actions are available. Say what actions are possible in the state `Poss(a,s)`.
These predicates have the form Precond(s)=> Poss(a,s). Preconditions based on state s imply what actions are
possible in state s.

*Successor state axioms* with fluent true being some fluent (In(c,p) for example)
```buildoutcfg
for all a,s Poss(a,s) => (fluent true <=> a made it true V a didnt make it false)
```

To describe initial state S0 and goal:

Simple predicates or simple predicates with a for all or exists ect.

Any first order logic prover can be used to solve these problems and much more flexible then simple planning.

## Local search algorithms and optimization problems

To get closer to real world problems, have to relax some assumptions.

Local search only looks at current states not explore new ones. Can use local search when solution is whats important
not the cost to get there.

Systematic search algorithms explore search spaces systematically by keeping one or more paths in memory and recording 
which alternatives have been explored at each point along the path. The path to the goal is the solution. 

In many problems, the path is not important only the goal.

*Local search algorithms: Use only the current single node. They generally move only to neighbors of that node. 
The path is usually not retained. 
They use very little memory and often find reasonable solutions in large
or infinite (continuous) state spaces where systematic algorithms don't work.

Local search algorithms can also optimise an objective function in an *optimisation problem*.

A *state-space landscape* has a location (state) and an elevation (heuristic cost / objective function). 
Local search algorithms traverse the state-space landscape searching for global max/mins.

A *complete* local search algorithm always finds a goal if one exists 
 
An *optimal* algorithm always finds a global minimum/maximum.

### Hill-climbing search

```buildoutcfg
function Hill-Climbing (problem):
    current <- Make-Node (problem.Initial-State)
    do:
        neightbor <- max(current.successors.value)
        if neighbor.value <= current.value:
            return current.state
        else current <- neighbor
    return current.state
```

This is the *steepest ascent* version of the hill-climbing search algorithm. It does not save a search tree. 
The data structure only needs to save the state and the value of the objective function for the current node. 

Local search algorithms typically use a complete-state formulation. 

Hill-climbing algorithms typically choose randomly among the set of best successors if there
is more than one.

Hill climbing is also called *greedy local search* (chooses the best neighbor without looking ahead)

Hill climbing works very well at bad states and can improve them quickly but can get stuck when:

1. There are *local maxima* which trap the search an prevent it from finding the global maximum.
2. There are *ridges* (sequence of local maxima)
3. There is a *plateau* (flat area of the state-space landscape). Either a flat local
maximum or a shoulder. These will stop a local search when states with no improvement in value of state causes
 the search to terminate.
 
This can be solved by allowing a *sideways move* and would reach a solution when the plateau is a shoulder.
This can cause an infinite loop in the case of a flat local maximum.
To stop an infinite loop we limit the number of consecutive sideways moves allowed. The cost is allowing extra moves,
making solutions longer.

*Stochastic hill climbing*
 
Selects a random uphill move with probability of selection dependent on the steepness of the move.
Converges slower than steepest ascent, but in some cases finds better solutions.

*First-choice hill climbing*
 
Generates successors randomly until one better than the current state is found.
Works well when a state has many possible successors.

All of these algorithms are incomplete because they might not find a possible solution when they get stuck on
on a local maximum.

*Random-restart hill climbing*
 
Performs many hill-climbing searches from randomly generated initial states, until a goal
is found. Complete with probability approaching 1. If probability p of
success, then the expected number of restarts required is 1/p.
The expected number of steps is the cost of one successful iteration plus
(1−p)/p times the cost of failure.

###Simulated annealing

Shakes up the algorithm enough to move out of a local max/min but not out of global max/min.

Any hill-climb which can't go downwards will always be incomplete (get stuck at a local max).
Random walk is complete but extremely inefficient.

Algorithm picks a random move. If the move improves the situation, it is always accepted. If not, 
the move is accepted with prob decreasing exponentially
the worse the move is.
The prob depends on the amount ΔE by which the evaluation is worsened and T (as T increases bad 
moves less likely to be accepted). But in the beginning when T is very large in the schedule, the probability 
of a bad move being accepted tends to 1. On a plateau, delts E is 0 so prob of accepting a worse state is 1
no matter how small T becomes.

When T is decreased slowly enough with a schedule (mapping from time to temperature)
, it always finds a global optimum with probability approaching 1.

```buildoutcfg
function Simulate-annealing(problem, schedule):
    current = Make-Node(problem.Initial-State)
    for t = 1 to inf:
        T = schedule(t)
        if T = 0:
            return current
        next = random(current.successors)
        deltaE = next.Value - current.Value
        if deltaE > 0 then:
            current = next
        else:
            prob = exp(deltaE/T)
            rand = rand(0,1)
            if rand <= prob:
                current = next
    return current
```

###Local beam search

Keeps track of k states rather than one node at a time. 
Generate k states randomly then at each step, successors of all k states are generated. 
Selects k best successors over the complete list and continues until one of them is a goal.

Useful information is shared between the parallel searches.

Can suffer from lack of diversity among the k states and get concentrated in a small region of the state space

*Stochastic beam search*

Instead of choosing the best k successors, they are chosen at random.
Prob of choosing a successor increases as its value increases.

*Genetic algorithms* 
A variant of stochastic beam search.
Successor states generated by combining two parent states rather than by modifying a single state.

Begin with k randomly generated states(the population). 
Each state is represented as a string over a finite alphabet

The *fitness function* is an objective function used to evaluate each state.
It returns higher values for better states.
Probability of being chosen for reproducing is directly proportional to the
fitness score. Your fitness score/ sum of fitness scores of all.
For a pair to be mated, a *crossover* point is chosen randomly from the positions in the string.
offspring are created by crossing over parent strings at the
crossover point. 

When parent states are quite different, the crossover operation can produce a state that is
a very different from either of them. 
This causes large steps in the state space early in the search process and smaller steps later on.

Each location is subject to random *mutation* with a small independent
probability in each offspring. 

Here uphill searching is combined with random exploration and parallel exchange of information.
 
A *schema* is a substring with positions that can be left without a value.
Strings matching the schema ara *instances* of it.

If the average fitness of all instances for a schema is above the mean, 
the number of instances of the schema within the population
will grow over time. 

Genetic algorithms work best when schemata correspond to meaningful
components of a solution.
 
```buildoutcfg
function Genetic_Algorithm (population, fitness_fn):
    do until any(population.fitness_fn >= F) or t >= T:
        new_pop = empty set
        for i=1 to Size(population):
            x = Random_select(population, fitness_fn)
            y = Random_select(population, fitness_fn)
            child = Reproduce(x,y)
            prob = small prob
            rand = rand(0,1)
            if rand <= prob:
                child = Mutate(child)
            new_pop.add(child)
        population = new_pop
    return population[max(population.fitness_fn),]
```

### Local search in continuous spaces

Only first-choice hill climbing and simulated annealing can be used when the state space is continuous.
These algorithms choose successors randomly, which can be done by generating random vectors of length δ.
The others have infinite branching factors.
 
Can *discretize* the neighborhood of each state to avoid being in a continuous prob.

States are defined by an n-dimensional vector of variables x.
Objective function f(x).

Can use *gradient methods* to find a maximum. 
Use gradient of the objective function ∇f -> partial derivative of objective.
∇f = (∂f/∂x1, ∂f/∂y1, ...).

In simple cases where closed form expression exists solution is to solve ∇f = 0. 

When no closed form, can calc gradient locally only. SO just ∂f/∂x1.

Given a locally correct expression for the gradient, we can perform steepest-ascent hill climbing by updating 
the current state: x ← x + α∇f(x) (α: step size). if α is too small, many steps are needed and if it
is too large, the search could overshoot the maximum.

When the objective function is not in a differentiable form we use the *empirical gradient* by evaluating the
response to small increments and decrements in each coordinate.

*Line search*

Helps with choice of step size. It extends the current gradient direction—usually by repeatedly
doubling α—until f starts to decrease again. The point at which this occurs becomes the new
current state. The new direction is then chosen. This can be accomplished with:

*Newton–Raphson method* computes estimates for the root x by solving g(x)=0 with x ← x − g(x)/g'(x) .
Max or min of objective is where gradient is 0: ∇f(x) = 0. So the update is x ← x − H^−1*∇f(x). But computing
all n^2 entries of hessian can be expensive.


Problems of  local maxima, ridges, and plateaux also exist in the continuous state spaces .
High-dimensional continuous spaces become very big and it is easy to get lost.

*Constrained optimization problems*

Solutions must satisfy hard constraints on the variables. Difficulty depends on type of constraints 
and objective function.

An example of these problems are *linear programming problems*. 
Constraints are linear inequalities forming a convex set. Objective function is also
linear. Complexity is polynomial in number of variables.

*Convex optimization* are the more general class of this problem. 

###SEARCHING WITH NONDETERMINISTIC ACTIONS

When the environment is either partially observable or nondeterministic you need to be able to perceive and learn from
environment as you go through. 

Every percept narrows down the set of possible states the agent might be in. 

In the nondeterministic case, percepts tell the agent which of the possible outcomes of its actions has actually occurred.
In the partially observable case, percepts tell the agent which possible state they are in.
 
Future percepts cannot be determined in advance and the future actions depend on these.

Solutions are not a sequence but a *contingency plan* or *strategy*. Plans which say what to do depending on the percepts.

To formulation nondeterministic problems, we need to generalize the notion of a transition model to be
a RESULT function that returns a set of possible outcome states instead of just one state.
Solutions can contain nested if–then–else statements to do this (more like a tree).

####AND–OR search trees

In deterministic environments, the only branching comes from the agents choices in each state (*OR nodes*).
In nondeterministic environments, branching can also come from the environment’s choice of outcome for each action
(*AND nodes*). AND and OR nodes then alternate to lead to an *AND–OR tree*.
 
A solution for an AND–OR search problem is a subtree that:
1. Has a goal node at every leaf
2. Specifies one action at each of its OR nodes
3. Includes every outcome branch at each of its AND nodes. 

The solution is shown in bold lines in the figure; it corresponds
to the plan given in Equation (4.3). (The plan uses if–then–else notation to handle the AND
branches, but when there are more than two branches at a node, it might be better to use a case

This can be solved with an *interleaving search* where the agent can act before it has found a guaranteed plan and 
deals with some contingencies only as they arise.
 
 Depth first search example:
 
 ```buildoutcfg
function AND-OR-GS (problem):
    OR-Search(problem.Initial_state, problem,[])
    return action|plan or failure
    
function OR-Search (state, problem, path):
    if problem.Goal_test(state):
        return empty_plan
    if state is on path:
        return failure
    for action in problem.Actions(state):
        plan = AND-Search(Results(state,action), problem, [state|path]
        if plan <> failure:
            return action|plan]
    return failure

function AND-Search(states,problem,path):
    for si in states:
        plani = OR-Search(si,problem,path)
        if plani = failure:
            return failure
    return [if si then plani,...,if sn-1 then plann-1, else plann]
```
 
Cycles often arise in nondeterministic problems. So if the current state is identical to a state on the path from 
the root, then it returns with failure. 
There could still be a solution from current state but any noncyclic solution must be reachable from the earlier
incarnation of the current state.
s that the algorithm terminates in every finite state space.

Can also be explored by breadth-first or best-first methods. The concept
of a heuristic function must be modified to estimate the cost of a contingent solution rather
than a sequence.

In problems where there is no acyclic solution but there is a cyclic one, 
the solution is expressed by adding a label to denote some portion of the plan and using that label later instead of 
repeating the plan itself. L1: while state=x do y.

Cyclic plans are a solution if:
1. Every leaf is a goal state 
2. A leaf is reachable from every point in the plan. 

Loop in the state space back to a state L is same as a loop in the plan back to the point where 
the subplan for state L is executed.

##Multiagent domains

###Adversarial search

Deterministic, turn-taking, two-player, *zero-sum games* (total payoff to all players
is the same in every game) of perfect information are just deterministic problems.
Fully observable environments in which two agents act alternately and
the utility values at the end of the game are equal and opposite. 


Games are too hard to solve. So need the ability to make some decision even when calculating the optimal
decision is infeasible. Games also penalize inefficiency severely. 

We use *pruning* to cut out portions of the search tree that make no difference to the final choice.
Heuristic functions approximate true utility without doing a complete search.
 
In a *max-min* game one agent wants to max and the other to min.
To define a game as a search problem we need:
1. S0: The initial state.
2. PLAYER(s): Defines which player has the move in a state.
3. ACTIONS(s): Returns the set of legal moves in a state.
4. RESULT(s, a): The transition model, which defines the result of a move.
5. *TERMINAL-TEST(s)*: A terminal test, which is true when the game is over. 
States where the game has ended are *terminal states*.
6. UTILITY(s, p): A utility function (also called an objective function or payoff function),
defines the final numeric value for a game that ends in terminal state s for a player p. 

Initial state, ACTIONS function, and RESULT function define the *game tree*. 


###OPTIMAL DECISIONS IN GAMES

Solution specifies MAX’s move in the initial state, then actions for every possible state that could be caused by MIN.
This is an AND–OR search.
 
Optimal strategies lead to outcomes at least as good as any other strategy when one is playing an
infallible opponent. 

A tree is one move deep if each player has only one action (two half moves each called a *ply*).

Given a game tree, the optimal strategy can be determined from the minimax value
of each node (utility for MAX assuming that both players play optimally till end of game from there -> *MINIMAX(n)*). 

MINIMAX(s) = UTILITY(s) if TERMINAL-TEST(s)
             max(MINIMAX(RESULT(s, a)) over all possible actions if PLAYER(s) = MAX
             min(MINIMAX(RESULT(s, a)) over all possible actions if PLAYER(s) = MIN
        
For max, the *minimax decision* at each node is the action that leads to the highest minimax utility.
     
####The minimax algorithm

The minimax algorithm performs a complete depth-first exploration of the game tree. Alternating between min and max nodes:

```python
def min_value(gameState):
    if gameState.terminal_test():
        return gameState.utility(0)
    v = float("inf")
    for a in gameState.actions():
        v = min(v, max_value(gameState.result(a)))
    return v


def max_value(gameState):
    if gameState.terminal_test():
        return gameState.utility(0)
    v = float("-inf")
    for a in gameState.actions():
        v = max(v, min_value(gameState.result(a)))
    return v

```

Then the actual minimax algorithm from the perspective of a max agent is:
```python
def minimax_decision(gameState):
    return max(gameState.actions(), key= lambda a: min_value(gameState.result(a)))
```
Max number of nodes will always be less than m! where m is number of blocks. Can't have more the m moves in a 
game so this is the mac depth of the tree.

If the maximum depth of the tree is m and there are b legal moves at each point (branching factor), then the
time complexity of the minimax algorithm is O(b^m). 
Can be estimated by average_branching_factor^average_number_nodes.
The space complexity is O(bm) if all actions generated once, or O(m) if one at a time.
 
####Optimal decisions in multiplayer games

Single value for each node becomes a vector.

The backed-up value of a node n is always the utility vector of the successor state with the highest value 
for the player choosing at n. 

###ALPHA–BETA PRUNING

Number of game states minimax has to go through is exponential in the depth of the tree. 
Can cut exponent in half if can compute the correct minimax decision without looking at every node.

*Alpha–beta pruning*

Returns the same move as minimax would, removes branches that can't influence the final decision because
they are guaranteed to be worse than the other solutions at a max node or better than other at a min node.
Can be applied to trees of any depth. 
 
For a node n, if the player has a better choice m at the parent node of n or at any choice point further up,
then n will never be reached in actual play. So it can br pruned out.
 
Two parameters are used to describe bounds on the backed-up values that appear anywhere
along the path:
1. α = the value of the best (i.e., highest-value) choice we have found so far at any choice point
along the path for MAX.
2. β = the value of the best (i.e., lowest-value) choice we have found so far at any choice point
along the path for MIN.

Alpha and beta values are updated along the search branches and prunes the remaining
branches at a node as soon as the value of the current
node is known to be worse than the current α or β value for MAX or MIN, respectively. 

```buildoutcfg
function ALPHA-BETA-SEARCH(state):
    v = MAX-VALUE(state,−∞,+∞)
    return action in ACTIONS(state) where action.value = v
    
function MAX-VALUE(state,α,β):
    if TERMINAL-TEST(state):
        return UTILITY(state)
    v = −∞
    for a in ACTIONS(state):
        v = MAX(v, MIN-VALUE(RESULT(s,a),α,β))
        if v ≥ β:
            return v
        α = MAX(α, v)
    return v
    
function MIN-VALUE(state,α,β):
    if TERMINAL-TEST(state):
        return UTILITY(state)
    v = +∞
    for a in ACTIONS(state):
        v = MIN(v, MAX-VALUE(RESULT(s,a) ,α,β))
        if v ≤ α:
            return v
        β = MIN(β, v)
    return v
```

####Move ordering

The effectiveness of alpha–beta pruning depends on order of how states are visited.
Need to first the successors that are likely to be best so worse ones can be pruned.

With correct ordering alpha–beta needs to examine only O(bm/2) nodes instead of O(bm).
This makes the effective branching factor √b instead of b
If states are seen in a random order, number of nodes examined will be roughly O(b3m/4).
 
Even simple ordering gets you close to the best-case O(bm/2). Dynamic move-ordering schemes, 
such as trying first the moves that were found
to be best in the past, gets even closer to the theoretical limit. 

One way to gain information from the current move is with iterative
deepening search:
1. Search 1 ply deep and record the best path of moves. 
2. Search 1 ply deeper, but use the recorded path to inform move ordering. 

Iterative deepening only adds a constant fraction to the total search time.
 
Best moves are called *killer moves* so this is called the killer move heuristic.

Repeated states can cause an exponential increase in search cost. 

Repeated states occur because of *transpositions*.
These are different permutations of the move sequence that end up in the same position. 

Can store the evaluation of the resulting position in a hash table the first time it is encountered so that
we don’t have to recompute it on subsequent occurrences. This table is called a *TRANSPOSITION TABLE*.
But can become impractical to keep many many states in the transposition table. 
Various strategies can be used to choose which nodes to keep and which to discard.