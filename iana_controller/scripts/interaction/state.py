from interaction.behavior.known_person_entered import KnownPersonEnteredBehavior, GreetKnownPerson
from interaction.behavior.known_person_left import KnownPersonLeftBehavior
<<<<<<< HEAD
from interaction.behavior.unknown_person_entered import UnknownPersonEnteredBehavior, GreetUnknownPerson, \
    RegisterUnknownPerson
=======
from interaction.behavior.state_enter import StateEnterBehaviour, ResumeTaskListOnStateEnter
from interaction.behavior.state_leave import StateLeaveBehaviour, InterruptTaskListOnStateLeave
from interaction.behavior.unknown_person_entered import UnknownPersonEnteredBehavior, GreetUnknownPerson
>>>>>>> 826f9da132c145f3eec6749c2ec20b76dc018638
from interaction.behavior.unknown_person_left import UnknownPersonLeftBehavior


class InteractionState(KnownPersonEnteredBehavior, KnownPersonLeftBehavior, UnknownPersonEnteredBehavior, UnknownPersonLeftBehavior, StateEnterBehaviour, StateLeaveBehaviour):
    pass


class IdleState(ResumeTaskListOnStateEnter, InterruptTaskListOnStateLeave, InteractionState):
    pass


class GreetingsState(GreetKnownPerson, RegisterUnknownPerson, InteractionState):
    pass
