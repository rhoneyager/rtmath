create table site (
 id   varchar(3) primary key,
 name varchar(80)
);
create index site_index_id on site(id);

create table subsite (
 id uuid primary key,
 subsite varchar(3), --- duplicates with sgp and twp do occur. also have E10.
 site varchar(3) references site(id),
 name varchar(80),
 lat real,  --- latitude (decimal degrees)
 lon real,  --- longitude (decimal degrees east)
 alt real   --- altitude (m)
);
create index subsite_index_id on subsite(id);
create index subsite_index_subsite on subsite(subsite);
create index subsite_index_site on subsite(site);

create table product ( --- swacr, vceil, ...
 name  varchar(80) primary key,
 fullname varchar(255)
);
create index product_index_name on product(name);

create table stream (
 id  uuid primary key,
 name  varchar(80),
 description varchar(255),
 product varchar(80) references product(name)
);

create index stream_index_id on stream(id);
create index stream_index_name on stream(name);
create index stream_index_product on stream(product);

create table datalevel (
 level varchar(10) primary key,
 description varchar(255)
);

create index datalevel_index_level on datalevel(level);

create table arm_info_obs (
 subsite_id uuid references subsite(id),
 stream_id  uuid references stream(id),
 datalevel  varchar(10) references datalevel(level),
 startTime  timestamp,
 endTime   timestamp,
 size   int, --- bytes
 filename  varchar(254) primary key,
 path   text
);

create view arm_info_obs_per_site as 
 select subsite.site, subsite.subsite, count(*) as numObs 
 from arm_info_obs, subsite 
 where subsite.id = arm_info_obs.subsite_id 
 group by subsite.site, subsite.subsite 
 order by subsite.site, subsite.subsite;
grant select on arm_info_obs_per_site to public;


create view arm_info as select
 subsite.site, subsite.subsite, 
 product.name as product, stream.name as stream, 
 arm_info_obs.datalevel, 
 arm_info_obs.filename, arm_info_obs.size, arm_info_obs.path,
 arm_info_obs.startTime, arm_info_obs.endTime
 from subsite, product, stream, arm_info_obs
 where subsite.id = arm_info_obs.subsite_id
 and stream.id = arm_info_obs.stream_id
 and product.name = stream.product
 order by subsite.site, subsite.id, product.name, stream.name, 
 arm_info_obs.datalevel, arm_info_obs.startTime
 ;

create index arm_info_obs_index_subsite on arm_info_obs(subsite_id);
create index arm_info_obs_index_stream on arm_info_obs(stream_id);
create index arm_info_obs_index_datalevel on arm_info_obs(datalevel);
create index arm_info_obs_index_starttime on arm_info_obs(startTime);

create view arm_info_site_stream_obs as
 select subsite.site, subsite.subsite, 
 stream.product as prod, stream.name as stream, 
 arm_info_obs.datalevel, count(*) as numObs 
 from arm_info_obs, subsite, stream
 where arm_info_obs.subsite_id = subsite.id
 and arm_info_obs.stream_id = stream.id
 group by subsite.site, subsite.subsite, stream.product, stream.name, arm_info_obs.datalevel
 order by subsite.site, subsite.subsite, stream.product, stream.name, arm_info_obs.datalevel;



create table arm_interval_obs (
 startFile varchar(254) references arm_info_obs(filename) primary key, 
 endFile varchar(254) references arm_info_obs(filename) unique not null
);

create view arm_interval as 
with startf as (select * from arm_info_obs, arm_interval_obs where filename = arm_interval_obs.startFile),
 endf as (select * from arm_info_obs, arm_interval_obs where filename = arm_interval_obs.endFile)
select
 subsite.site, subsite.subsite,
 product.name as product, stream.name as stream,
 startf.datalevel,
 startf.startFile, startf.startTime, startf.endTime - startf.startTime as startDuration,
 endf.endFile, endf.endTime, endf.endTime - endf.startTime as endDuration, 
 endf.endTime - startf.startTime as duration
 from subsite, product, stream, startf, endf, arm_interval_obs
 where subsite.id = startf.subsite_id
 and stream.id = startf.stream_id
 and product.name = stream.product
 and startf.filename = arm_interval_obs.startFile
 and endf.filename = arm_interval_obs.endFile
 order by subsite.site, subsite.subsite, stream, startf.datalevel, startf.startTime
 ;

 create view arm_interval_b as 
with startf as (select * from arm_info_obs, arm_interval_obs where filename = arm_interval_obs.startFile),
 endf as (select * from arm_info_obs, arm_interval_obs where filename = arm_interval_obs.endFile)
select
 subsite.site, subsite.subsite as sub, stream.name as stream, startf.datalevel as dl,
 tsrange(startf.startTime, endf.endTime) as inter,
 endf.endTime - startf.startTime as duration, 
 startf.endTime - startf.startTime as startDur,
 endf.endTime - endf.startTime as endDur
 from subsite, stream, startf, endf, arm_interval_obs
 where subsite.id = startf.subsite_id
 and stream.id = startf.stream_id
 and startf.filename = arm_interval_obs.startFile
 and endf.filename = arm_interval_obs.endFile
 order by subsite.site, subsite.subsite, stream, startf.datalevel, startf.startTime
 ;

 create view arm_interval_unpaired as
 with matchers as ( select filename, site, subsite, stream, datalevel, startTime, endTime, tsrange(arm_info.startTime, arm_info.endTime) && any 
      (select inter from arm_interval_b where site = arm_info.site and sub = arm_info.subsite and stream = arm_info.stream and dl = arm_info.datalevel) as inint 
      from arm_info )
 select site, subsite as sub, stream, datalevel as dl, filename from matchers where inint = false
 order by site, subsite, stream, datalevel, startTime;

 create view arm_interval_paired as
 with matchers as ( select filename, site, subsite, stream, datalevel, startTime, endTime, tsrange(arm_info.startTime, arm_info.endTime) && any 
      (select inter from arm_interval_b where site = arm_info.site and sub = arm_info.subsite and stream = arm_info.stream and dl = arm_info.datalevel) as inint 
      from arm_info )
 select site, subsite as sub, stream, datalevel as dl, filename from matchers where inint = true
 order by site, subsite, stream, datalevel, startTime;


 create view arm_interval_can_match as
 with matchers as ( select filename, site, subsite, stream, datalevel, startTime, endTime,
      tsrange(arm_info.startTime, arm_info.endTime)
      && any (select tsrange(lower(inter) - duration / 20, upper(inter) + duration / 20) 
	  from arm_interval_b where site = arm_info.site and sub = arm_info.subsite
	  and stream = arm_info.stream and dl = arm_info.datalevel) as canmatch from arm_info )
 select site, subsite as sub, stream, datalevel as dl, filename from matchers where canmatch = true
 except select site, sub, stream, dl, filename from arm_interval_paired
 order by site, sub, stream, dl
 ;

--- TODO!
create view arm_interval_adjacent_intervals as
 with matchers as ( select filename, site, subsite, stream, datalevel, startTime, endTime,
      tsrange(arm_info.startTime, arm_info.endTime)
      && any (select tsrange(lower(inter) - duration / 20, upper(inter) + duration / 20) 
	  from arm_interval_b where site = arm_info.site and sub = arm_info.subsite
	  and stream = arm_info.stream and dl = arm_info.datalevel) as canmatch from arm_info )
 select site, subsite as sub, stream, datalevel as dl, filename from matchers where canmatch = true
 except select site, sub, stream, dl, filename from arm_interval_paired
 order by site, sub, stream, dl
 ;

 grant select on arm_interval_unpaired to public;
 grant select on arm_interval_obs to public;
 grant select on arm_interval to public;
 grant select on arm_info TO public;
 grant select on site TO public;
 grant select on subsite TO public;
 grant select on product TO public;
 grant select on stream TO public;
 grant select on datalevel TO public;
 grant select on arm_info_obs TO public;

 grant all on arm_interval_obs TO rtmath_test;
 grant all on arm_interval TO rtmath_test;
 grant all on arm_info TO rtmath_test;
 grant all on site TO rtmath_test;
 grant all on subsite TO rtmath_test;
 grant all on product TO rtmath_test;
 grant all on stream TO rtmath_test;
 grant all on datalevel TO rtmath_test;
 grant all on arm_info_obs TO rtmath_test;

