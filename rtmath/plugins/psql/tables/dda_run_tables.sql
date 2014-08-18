CREATE TABLE host (
	id				uuid primary key,
	hostname		varchar(80),
	os				varchar(80),
	active			bool
);

CREATE TABLE ddaVersion (
	id				uuid primary key,
	fullName		varchar(255),
	version			int,
	active			bool
);

CREATE TABLE flakeTypes (
	id				uuid primary key,
	name			varchar(80),
	description		varchar(255)
);

CREATE TYPE activeRunProgress AS ENUM (
	'created', 
	'running',
	'finished',
	'ingested',
	'cancelled'
);

CREATE TABLE activeRuns (
	id				uuid primary key,
	tsCreated		timestamp with time zone,
	tsLastStarted	timestamp with time zone,
	tsLastUpdated	timestamp with time zone,
	tsFinished		timestamp with time zone,
	tsTransferred	timestamp with time zone,
	frequency		real, --- frequency of run (GHz)
	temperature		real, --- temperature of run (K)
	decimation		varchar(80),
	perturbation	varchar(80),
	ddaVersion		uuid references ddaVersion(id),
	polarization	varchar(9),
	flakeType		uuid references flakeTypes(id),
	host			uuid references host(id),
	path			varchar(1024),
	runsTotal		int,
	runsCompleted	int,
	runsFailed		int,
	progress		activeRunProgress,
	nBetas			int,
	nThetas			int,
	nPhis			int,
	description		varchar(1024),
	username		varchar(80)
);

CREATE VIEW stalledRuns AS
	SELECT 
	activeRuns.username,
	activeRuns.tsLastStarted, activeRuns.tsLastUpdated, 
	host.hostname,
	activeRuns.path,
	activeRuns.runsTotal, activeRuns.runsCompleted, activeRuns.runsFailed
	FROM activeRuns, host, flakeTypes
	WHERE age(activeRuns.tsLastUpdated) > '2 days'
	AND activeRuns.progress = 'running'
	AND host.id = activeRuns.host
	AND flakeTypes.id = activeRuns.flakeType
	ORDER BY activeRuns.tsLastUpdated;

CREATE VIEW successfulRuns AS
	SELECT
	activeRuns.username,
	flakeTypes.name,
	activeRuns.frequency,
	activeRuns.temperature,
	host.hostname,
	activeRuns.path,
	activeRuns.decimation,
	activeRuns.perturbation,
	activeRuns.polarization,
	activeRuns.runsTotal,
	activeRuns.nBetas, activeRuns.nThetas, activeRuns.nPhis, 
	activeRuns.tsFinished
	FROM activeRuns, host, flakeTypes
	WHERE activeRuns.progress = 'finished'
	AND host.id = activeRuns.host
	AND activeRuns.runsFailed = 0
	AND flakeTypes.id = activeRuns.flakeType
	ORDER BY activeRuns.tsFinished
	;

CREATE VIEW failedRuns AS
	SELECT
	activeRuns.username,
	flakeTypes.name,
	activeRuns.frequency,
	activeRuns.temperature,
	host.hostname,
	activeRuns.path,
	activeRuns.decimation,
	activeRuns.perturbation,
	activeRuns.polarization,
	activeRuns.runsTotal, activeRuns.runsCompleted, activeRuns.runsFailed,
	activeRuns.nBetas, activeRuns.nThetas, activeRuns.nPhis, 
	activeRuns.tsFinished
	FROM activeRuns, host, flakeTypes
	WHERE activeRuns.progress = 'finished'
	AND activeRuns.runsFailed > 0
	AND host.id = activeRuns.host
	AND flakeTypes.id = activeRuns.flakeType
	ORDER BY activeRuns.tsFinished
	;

CREATE VIEW currentRuns AS
	SELECT
	activeRuns.username,
	flakeTypes.name,
	activeRuns.frequency,
	activeRuns.temperature as temp,
	host.hostname,
	activeRuns.path,
	activeRuns.decimation as dec,
	activeRuns.perturbation,
	activeRuns.polarization as pol,
	activeRuns.runsTotal as total, activeRuns.runsCompleted as good, activeRuns.runsFailed as bad,
	activeRuns.nBetas, activeRuns.nThetas, activeRuns.nPhis, 
	activeRuns.tsLastStarted, 
	activeRuns.tsLastUpdated
	FROM activeRuns, host, flakeTypes
	WHERE activeRuns.progress = 'running'
	AND host.id = activeRuns.host
	AND flakeTypes.id = activeRuns.flakeType
	ORDER BY host.hostname, activeRuns.runsCompleted
	;

CREATE VIEW waitingRuns AS
	SELECT
	activeRuns.username,
	flakeTypes.name,
	activeRuns.frequency,
	activeRuns.temperature,
	host.hostname,
	activeRuns.path,
	activeRuns.decimation,
	activeRuns.perturbation,
	activeRuns.polarization,
	activeRuns.runsTotal,
	activeRuns.nBetas, activeRuns.nThetas, activeRuns.nPhis, 
	activeRuns.tsCreated
	FROM activeRuns, host, flakeTypes
	WHERE activeRuns.progress = 'created'
	AND host.id = activeRuns.host
	AND flakeTypes.id = activeRuns.flakeType
	ORDER BY host.hostname, activeRuns.tsCreated
	;

CREATE VIEW hostLoad AS
	SELECT host.hostname, count(currentRuns.hostname) 
	as NumberOfRunningJobs from host
	left join currentRuns on currentRuns.hostname = host.hostname 
	and host.active = true
	group by host.hostname
	order by host.hostname;


	grant select on currentruns TO public;
	grant select on failedruns TO public;
	grant select on stalledruns TO public;
	grant select on successfulruns TO public;
	grant select on waitingruns TO public;
	grant select on hostLoad TO public;

	grant insert ON activeruns TO ddastatus_updater;
	grant update ON activeruns TO ddastatus_updater;
	grant select on ddaversion to ddastatus_updater ;
	grant select on host to ddastatus_updater ;
	grant select on flaketypes to ddastatus_updater ;
	grant select ON activeruns to ddastatus_updater ;
