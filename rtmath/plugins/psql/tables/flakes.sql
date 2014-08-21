--- These tables reference flakeTypes, found in dda_run_tables.sql

create table flake (
	hashLower varchar(80) primary key,
	hashUpper varchar(80) unique,
	flakeType_id uuid references flakeTypes(id),
	tags text[],
	standardD real,
	numDipoles int,
	description varchar(1023),
	flake_references varchar(80) references flake(hashLower)
);
create index flake_group on flake(flakeType_id);
create index flake_upperhash on flake(hashUpper);
create index flake_dipoles on flake(numDipoles);

grant select on flake TO public;
grant insert ON flake TO ddastatus_updater;
grant update ON flake TO ddastatus_updater;

create view flake_summary as 
 select flake.hashLower,
 flakeTypes.name,
 flake.tags,
 flake.numDipoles,
 flake.standardD,
 flake.flake_references
 from flake, flakeTypes
 where flake.flakeType_id = flakeTypes.id
 order by flakeTypes.name, flake.numDipoles
 ;
grant select on flake_summary to public;

create view flake_counts as 
 select name, count(*) 
 from flake_summary 
 group by name order by name
 ;
grant select on flake_counts to public;
