create table site (
	id 		varchar(3) primary key,
	name	varchar(80)
);

create table subsite (
	id		varchar(2) primary key,
	site	varchar(3) references site(id),
	fullname	varchar(80),
	lat		real,		--- latitude (decimal degrees)
	lon		real,		--- longitude (decimal degrees east)
	alt		real 		--- altitude (m)
);

create table product ( --- swacr, vceil, ...
	name 	varchar(80) primary key,
	fullname varchar(255)
);

create table stream (
	id 	uuid primary key,
	name 	varchar(80),
	description varchar(255),
	product varchar(80) references product(name)
);

create table datalevel (
	level	varchar(10) primary key,
	description	varchar(255)
);

create table arm_info_obs (
	subsite varchar(2) references subsite(id),
	stream_id 	uuid references stream(id),
	datalevel 	varchar(10) references datalevel(level),
	startTime 	timestamp,
	endTime	 	timestamp,
	size 		int,	--- bytes
	filename 	varchar(80) primary key,
	path 		text
);

create view arm_info as select
	subsite.site as site_id, subsite.id as subsite_id, 
	product.name as product_name, stream.name as stream_name, 
	arm_info_obs.datalevel, 
	arm_info_obs.filename, arm_info_obs.size, arm_info_obs.path,
	arm_info_obs.startTime, arm_info_obs.endTime
	from subsite, product, stream, arm_info_obs
	where subsite.id = arm_info_obs.subsite
	and stream.id = arm_info_obs.stream_id
	and product.name = stream.product
	order by subsite.site, subsite.id, product.name, stream.name, arm_info_obs.datalevel,
	arm_info_obs.startTime
	;

	grant select on arm_info TO public;
	grant select on site TO public;
	grant select on subsite TO public;
	grant select on product TO public;
	grant select on stream TO public;
	grant select on datalevel TO public;
	grant select on arm_info_obs TO public;

