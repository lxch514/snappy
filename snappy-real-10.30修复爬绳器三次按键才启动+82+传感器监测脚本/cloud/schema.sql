PRAGMA journal_mode = WAL;

CREATE TABLE IF NOT EXISTS messages (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  robot_id TEXT NOT NULL,
  topic TEXT NOT NULL,
  kind TEXT NOT NULL,
  received_at_ms INTEGER NOT NULL,
  payload TEXT NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_messages_robot_received
  ON messages(robot_id, received_at_ms);

CREATE TABLE IF NOT EXISTS latest_state (
  robot_id TEXT PRIMARY KEY,
  received_at_ms INTEGER NOT NULL,
  payload TEXT NOT NULL
);
