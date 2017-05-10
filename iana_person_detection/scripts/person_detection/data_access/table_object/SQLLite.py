from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool

engine_instance = create_engine('sqlite:///storage/database.db',
    connect_args={'check_same_thread': False},
    poolclass=StaticPool, echo=False)

session_maker = sessionmaker(bind=engine_instance)