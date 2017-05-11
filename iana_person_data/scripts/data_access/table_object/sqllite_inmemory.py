from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool

engine_instance = create_engine('sqlite:///:memory:',
    connect_args={'check_same_thread': False},
    poolclass=StaticPool, echo=True)

session_maker = sessionmaker(bind=engine_instance)