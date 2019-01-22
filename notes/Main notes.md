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
Only things that are neccessary to know to complete the task.
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
correct h function to do so. This happens when h is optimistic/admissable.
-Finds lowest cost path when h(s)<true cost to get to goal from s.
h should not overestimate the distance to the goal.
This is because once path to goal is found, it is lower in cost than any other
on the frontier and we know its true cost since h=0 at goal. Also everything on
frontier will be understimating their cost to the goal and we are already
lower than that anyway. Also all steps are thought to have non zero cost.
So cost should keep going up for other paths.

--An admissable heuristic never over estimates the cost to get to the goal. 
An A* search with one heuristic that is more costly (>=) than another 
but still admissable will expand fewer paths to get to the goal.
--A consistent heuristic will never decrease cost by more than one for an action
which has a cost of 1.
--Heuristics can be combined so that new=max(old1,old2). Will be admissable as
long as both old ones are admissable and will expand fewer paths.
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