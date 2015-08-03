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

CREATE TABLE flakeRuns (
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
	ingestPath		varchar(1024),
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
	flakeRuns.username,
	flakeRuns.tsLastStarted, flakeRuns.tsLastUpdated, 
	host.hostname,
	flakeRuns.path,
	flakeRuns.runsTotal, flakeRuns.runsCompleted, flakeRuns.runsFailed
	FROM flakeRuns, host, flakeTypes
	WHERE age(flakeRuns.tsLastUpdated) > '3 days'
	AND flakeRuns.progress = 'running'
	AND host.id = flakeRuns.host
	AND flakeTypes.id = flakeRuns.flakeType
	ORDER BY flakeRuns.tsLastUpdated;

CREATE VIEW successfulRuns AS
	SELECT
	flakeRuns.username,
	flakeTypes.name,
	flakeRuns.frequency,
	flakeRuns.temperature,
	host.hostname,
	flakeRuns.path,
	flakeRuns.decimation,
	flakeRuns.perturbation,
	flakeRuns.polarization,
	flakeRuns.runsTotal,
	flakeRuns.nBetas, flakeRuns.nThetas, flakeRuns.nPhis, 
	flakeRuns.tsFinished
	FROM flakeRuns, host, flakeTypes
	WHERE flakeRuns.progress = 'finished'
	AND host.id = flakeRuns.host
	AND flakeRuns.runsFailed = 0
	AND flakeTypes.id = flakeRuns.flakeType
	ORDER BY flakeRuns.tsFinished
	;

CREATE VIEW failedRuns AS
	SELECT
	flakeRuns.username,
	flakeTypes.name,
	flakeRuns.frequency,
	flakeRuns.temperature,
	host.hostname,
	flakeRuns.path,
	flakeRuns.decimation,
	flakeRuns.perturbation,
	flakeRuns.polarization,
	flakeRuns.runsTotal, flakeRuns.runsCompleted, flakeRuns.runsFailed,
	flakeRuns.nBetas, flakeRuns.nThetas, flakeRuns.nPhis, 
	flakeRuns.tsFinished
	FROM flakeRuns, host, flakeTypes
	WHERE flakeRuns.progress = 'finished'
	AND flakeRuns.runsFailed > 0
	AND host.id = flakeRuns.host
	AND flakeTypes.id = flakeRuns.flakeType
	ORDER BY flakeRuns.tsFinished
	;

CREATE VIEW currentRuns AS
	SELECT
	flakeRuns.username,
	flakeTypes.name,
	flakeRuns.frequency,
	flakeRuns.temperature as temp,
	host.hostname,
	flakeRuns.path,
	flakeRuns.decimation as dec,
	flakeRuns.perturbation,
	flakeRuns.polarization as pol,
	flakeRuns.runsTotal as total, flakeRuns.runsCompleted as good, flakeRuns.runsFailed as bad,
	flakeRuns.nBetas, flakeRuns.nThetas, flakeRuns.nPhis, 
	flakeRuns.tsLastStarted, 
	flakeRuns.tsLastUpdated
	FROM flakeRuns, host, flakeTypes
	WHERE flakeRuns.progress = 'running'
	AND host.id = flakeRuns.host
	AND flakeTypes.id = flakeRuns.flakeType
	ORDER BY host.hostname, flakeRuns.runsCompleted
	;

CREATE VIEW ingestedRuns AS
	SELECT
	flakeRuns.username,
	flakeTypes.name,
	flakeRuns.frequency,
	flakeRuns.temperature as temp,
	host.hostname,
	flakeRuns.decimation as dec,
	flakeRuns.perturbation,
	flakeRuns.polarization as pol,
	flakeRuns.runsTotal as total, 
	flakeRuns.nBetas, flakeRuns.nThetas, flakeRuns.nPhis, 
	flakeRuns.tsTransferred
	FROM flakeRuns, host, flakeTypes
	WHERE flakeRuns.progress = 'ingested'
	AND host.id = flakeRuns.host
	AND flakeTypes.id = flakeRuns.flakeType
	ORDER BY flakeRuns.frequency, temp, dec, flakeruns.perturbation
	;

CREATE VIEW waitingRuns AS
	SELECT
	flakeRuns.username,
	flakeTypes.name,
	flakeRuns.frequency,
	flakeRuns.temperature,
	host.hostname,
	flakeRuns.path,
	flakeRuns.decimation,
	flakeRuns.perturbation,
	flakeRuns.polarization,
	flakeRuns.runsTotal,
	flakeRuns.nBetas, flakeRuns.nThetas, flakeRuns.nPhis, 
	flakeRuns.tsCreated
	FROM flakeRuns, host, flakeTypes
	WHERE flakeRuns.progress = 'created'
	AND host.id = flakeRuns.host
	AND flakeTypes.id = flakeRuns.flakeType
	ORDER BY host.hostname, flakeRuns.tsCreated
	;

CREATE VIEW hostLoad AS
	SELECT host.hostname, count(currentRuns.hostname) 
	as NumberOfRunningJobs 
	from host
	left join currentRuns on currentRuns.hostname = host.hostname 
	and host.active = true
	group by host.hostname
	order by host.hostname;

CREATE VIEW flakeRuns_db AS
	SELECT
	flakeRuns.id, 
	flakeRuns.username,
	flakeTypes.name,
	flakeRuns.frequency,
	flakeRuns.temperature as temp,
	host.hostname,
	flakeRuns.path,
	flakeRuns.decimation as dec,
	flakeRuns.perturbation,
	flakeRuns.polarization as pol,
	flakeRuns.runsTotal as total, flakeRuns.runsCompleted as good, flakeRuns.runsFailed as bad,
	flakeRuns.progress,
	flakeRuns.nBetas, flakeRuns.nThetas, flakeRuns.nPhis, 
	flakeRuns.tsLastStarted, 
	flakeRuns.tsLastUpdated 
	FROM flakeRuns, host, flakeTypes 
	WHERE host.id = flakeRuns.host 
	AND flakeTypes.id = flakeRuns.flakeType 
	ORDER BY host.hostname, flakeRuns.runsCompleted
	;

	create view flakeRunsHiRes as select frequency, temp, name  from flakeruns_db where dec = 'none' and perturbation = 'none' and pol = 'lin' and (name like '%late%' or name like '%micron') and progress = 'ingested' and nbetas = 18 order by frequency, name;


	create view currentruns_console as select name, frequency as freq, temp, hostname as host, dec, perturbation as pert, pol, total, good, nbetas from currentruns order by total - good;


CREATE VIEW activeruns_console AS
	SELECT
	flakeTypes.name,
	flakeRuns.frequency as freq,
	flakeRuns.temperature as temp,
	host.hostname as host,
	flakeRuns.decimation as dec,
	flakeRuns.perturbation as pert,
	flakeRuns.polarization as pol,
	flakeRuns.runsTotal as total, flakeRuns.runsCompleted as good, flakeRuns.runsFailed as bad,
	flakeRuns.nBetas as nb, 
	flakeRuns.progress as state,
---age(current_timestamp, flakeRuns.tsLastUpdated)
	to_char( age(current_timestamp, flakeRuns.tsLastUpdated), 'HH24 "h"') as last
---extract(hour from age(current_timestamp, flakeRuns.tsLastUpdated) )
	FROM flakeRuns, host, flakeTypes
	WHERE (flakeRuns.progress = 'running' OR flakeRuns.progress = 'finished')
	AND host.id = flakeRuns.host
	AND flakeTypes.id = flakeRuns.flakeType
	ORDER BY flakeRuns.runsTotal - flakeRuns.runsCompleted
	;

	create view ingestedruns_console as select name, frequency as freq, temp, hostname as host, dec, perturbation as pert, pol, total, good, nbetas from ingestedruns order by total - good;

	grant select on flakeRunsHiRes to public;
	grant select on currentruns_console to public;
	grant select on activeruns_console to public;
	grant select on currentruns TO public;
	grant select on failedruns TO public;
	grant select on stalledruns TO public;
	grant select on successfulruns TO public;
	grant select on waitingruns TO public;
	grant select on hostLoad TO public;
	grant select on flakeRuns_db to public ;

	grant insert ON flakeRuns TO ddastatus_updater;
	grant update ON flakeRuns TO ddastatus_updater;
	grant select on ddaversion to ddastatus_updater ;
	grant select on host to ddastatus_updater ;
	grant select on flaketypes to ddastatus_updater ;
	grant select ON flakeRuns to ddastatus_updater ;
