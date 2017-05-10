from interaction.behavior.known_person_entered import KnownPersonEnteredBehavior, GreetKnownPerson
from interaction.behavior.known_person_left import KnownPersonLeftBehavior
from interaction.behavior.unknown_person_entered import UnknownPersonEnteredBehavior, GreetUnknownPerson
from interaction.behavior.unknown_person_left import UnknownPersonLeftBehavior


class InteractionState(KnownPersonEnteredBehavior, KnownPersonLeftBehavior, UnknownPersonEnteredBehavior, UnknownPersonLeftBehavior):
    pass


class IdleState(InteractionState):
    pass


class GreetingsState(GreetKnownPerson, GreetUnknownPerson, InteractionState):
    pass
