# Development guidelines

#### Table of Contents
- [Development branch](#default-development-branch)
- [Review process](#review-process)
- [Continuous integration](#continuous-integration-ci)
  - [How-to activate Travis for forked repositories](#how-to-activate-travis-for-forked-repositories)
  - [How-to run the CI processes locally](#how-to-run-the-ci-processes-locally)
- [Code style guide](#code-style-guide)
  - [Naming conventions](#naming-conventions)
  - [How-to format code automatically](#how-to-format-code-automatically)
- [Coverage](#coverage)
  - [How-to check coverage in C++ and Python](#how-to-check-coverage-in-c-and-python)
- [Specification](#specification)
  - [How-to define and write a requirement](#how-to-define-and-write-a-requirement)
  - [How-to link a requirement against a test](#how-to-link-a-requirement-against-a-test)
- [Documentation](#documentation)
  - [How-to generate documentation for a package](#how-to-generate-documentation-for-a-package)
- [Testing](#testing)
  - [How-to document tests](#how-to-document-tests)
  - [Obtain the logging output](#obtaining-the-logging-output)
    - [How-to obtain logging output for rostests](#how-to-obtain-logging-output-for-rostests)
    - [How-to change logging level for a package](#how-to-change-logging-level-for-a-package)
  - [How-to write proper tests](#how-to-write-proper-tests)
  - [How-to deal with threads in tests](#how-to-deal-with-threads-in-tests)
  - [Gtest command line flags](#gtest-command-line-flags)
  - [How-to run particular test of test suite](#how-to-run-particular-test-of-test-suite)
- [Mocking the hardware](#mocking-the-hardware)
- [FAQ](#faq)

## Default development branch
Our current default development branch is melodic-devel 
(see [here](https://github.com/PilzDE/pilz_robots/branches)).
All new feature are developed for 
[ROS Melodic Morenia](https://www.ros.org/reps/rep-0003.html#melodic-morenia-may-2018-may-2023) 
which primarily targets Ubuntu Bionic (18.04) (for more information see 
[here](http://wiki.ros.org/melodic)).  
For backwards compatibility reasons we still support 
[ROS Kinetic Kame](https://www.ros.org/reps/rep-0003.html#kinetic-kame-may-2016-may-2021)
in combination with Ubuntu Xenial (16.04).
In order to support ROS Kinetic Kame, we proceed as follows:
- First, we develop the new feature for ROS Melodic Morenia and merge the
new feature into the melodic-devel branch.
- Secondly, we cherry-pick the new feature from the melodic-devel branch and
merge it into the kinetic-devel branch.


## Review process
Before any pull request is approved a developer not involved in 
the development of the feature has to review the pull request.  
  
Our goal is to automate the review process as much as possible by including
as many checks as possible into the continuous integration process.
However, some things are currently manual or simply cannot be automated.  
In general the reviewer has to check/review the following:
- Specification:
  - The feature and its changes to the system are comprehensively specified.
  - All requirements are exhaustively tested by one or more tests.
  - All specifications are linked against their corresponding tests 
(how-to link see [here](#how-to-link-a-requirement-against-a-test))
- User interface:
  - The user interface is easy to understand and use.
  - The user interface is comprehensively documented.
  - If an existing user interface is broken by the new feature, then the
interface version has to be adapted according the nature of the change.
- System architecture:
  - The system architecture complies with the clean architecture guidelines.
(see "Clean Architecture" by Robert C. Martin)
  - The architecture diagrams are up to date and easily understandable.
- Code:
  - The code has to comply with the clean code rules 
(see "Clean Code: A Handbook of Agile Software Craftsmanship" by Robert C. Martin).
- Tests:
  - The tests are easy to understand (e.g. each test only checks one aspect 
of a component, etc.)
  - The tests are comprehensively documented (how-to document tests see
[here](#how-to-document-tests))


## Continuous integration (CI)
All pull requests (PR's) have to be approved by the continuous integration tool
[Travis](https://travis-ci.org/PilzDE/).  

### How-to activate Travis for forked repositories
If you are an external developer and want to create a pull request, you can
do so by forking one of our repositories, make changes, and then create a pull
request on our repository.  
To check that the continuous integration runs without problems on 
*your* forked repository, you have to activate Travis for your repository.
For more information on how to do that have a look at the
[Travis tutorials](https://docs.travis-ci.com/user/tutorial/#to-get-started-with-travis-ci).  
  
**Note:**  
In our repositories already exists a `.travis.yml` file, so there is no need
to create your own.

### How-to run the CI processes locally
If you have a look at our
[Travis file](https://github.com/PilzDE/pilz_robots/blob/melodic-devel/.travis.yml#L34), 
you see that our CI is based on the 
[ROS industrial CI configurations](https://github.com/ros-industrial/industrial_ci)
for ROS. In some cases, for example in case of errors only appearing on 
Travis, etc., it might be advantageous to do *exactly* what Travis does locally 
(At least in terms of building the packages and executing the tests).
To achieve this, have a look at the explanations given in the 
[ros-industrial repository](https://github.com/ros-industrial/industrial_ci/blob/legacy/doc/index.rst#run-industrial_ci-on-local-host).

## Code style guide
<!---TODO: Are we really compliant with the ROS coding style guide? --->
We follow the [ROS coding style guide](http://wiki.ros.org/CppStyleGuide). 
Currently, the styling of the code is not automatically checked by the CI, 
so please make sure that you comply with the ROS coding style guide. 

### Naming conventions
TODO

### How-to format code automatically
If you want to automatically format your code using the ROS style guide,
then see [here](http://wiki.ros.org/CppStyleGuide#Autoformatting_of_ROS_Code).

## Coverage
Only PR's are accepted which have a 100% line and function coverage. 
Currently the coverage check is only partially integrated into the CI.
Therefore, everybody has to ensure that all PR's fulfill the coverage 
requirement. Otherwise the reviewer has to reject the PR.

### How-to check coverage in C++ and Python
To manually run the coverage program for a certain package, the following steps
need to be performed:
- Delete the build and devel folder, to ensure a clean build.
- Run `catkin_make -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug`, 
to build all repositories first. This ensures that all necessary dependencies 
are build.
- Run `catkin_make -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug package_name_coverage`,
to build the concrete package and to execute all tests of the package.  
- Finally you can open the html page mentioned at the end of the console output.
The html page shows the summary of the coverage data collected during the 
execution of the tests.

**Note:**  
Make sure that all the tests are executed successfully because, otherwise,
the coverage value does not reflect the correct coverage of your tests.

## Specification
If new properties are added to the system or if old ones have to be changed, 
the specification of the system needs to be updated. 
The specification helps each one of us to quickly gain an understanding what 
and what the system cannot do.  
Currently, our requirements are situated in a file called `spec.dox`. 
To unify and simplify the creation of requirements, and the linking of
requirements against tests, we introduced new doxygen commands which are
explained in the following sub-sections.

**Please note:**  
The specification of a package is generated while generating the 
documentation. For more information on how to generate the documentation for
a package see section:
[How-to generate documentation for a package](#how-to-generate-documentation-for-a-package)

### How-to define and write a requirement
To define a requirement the following doxygen command is used:
```
@definereq{my_requirements_name}
Here comes the specification of my super cool new feature I'm adding.
```
**Some remarks how to formulate a requirement:**  
In order, to keep the requirement simple and understandable, use short and
plain sentences. If it difficult to deduce why the system needs to fulfill
a certain requirement, please shortly state the reason. Also, do not forget to
define how the system behaves if something goes wrong 
(e.g. errors, etc.) or for edge cases (e.g. lowest/highest possible
number, etc.)!  

### How-to link a requirement against a test
To state that a test checks a certain requirement or part of a requirement,
the following doxygen command is used:
```
@tests{my_requirement_name, A one sentence long short description of the test (without comma!)}
```
**Note:**
- A requirement is allowed to be tested by more than one test. In other words,
a requirement can be linked against as many tests as needed.
- A test is allowed to test more than on requirement. In other words, you are
allowed to use as many `@tests` commands in the test description as needed.

## Documentation
To document the code, [Doxygen](http://www.doxygen.nl/) is used for C++ code
and [Sphinx](https://www.sphinx-doc.org/en/master/index.html) for
Python code.

### How-to generate documentation for a package
To generate the documentation for a package consisting of C++ or/and 
Python code, a tool named [rosdoc_lite](http://wiki.ros.org/rosdoc_lite)
is used:
- How to install the tool see the ROS wiki:
[rosdoc_lite Installation](http://wiki.ros.org/rosdoc_lite#Installation). 
- How to use rosdoc_lite see the ROS wiki:
 [Generate document locally](http://wiki.ros.org/rosdoc_lite#Generate_document_locally).

## Testing

### How-to document tests
In order to keep the test documentation easily understandable,
we separate the test documentation in two parts:  
  
The first part summarizes the objective of the test. Please make sure to only
use one short sentence to summarize the objective. If the test has to be linked
against a requirement, make sure to put the summarizing sentence into the
doxygen test specification command. On how-to do that see the 
[requirement linking section](#how-to-link-a-requirement-against-a-test).  
  
The second part is a detailed step by step description what the test does and
what the expected result of the respective step is.
To do so, we use the following scheme:
```
Test Sequence:
  0. Explain in one short sentence what is done in step 1.
  1. Explain in one short sentence what is done in step 2.
  ...

Expected Results:
  0. Explain in one short sentence what result you expect in step 1.
  1. Explain in one short sentence what result you expect in step 2.
  ...

```
For a proper test documentation please do not forget to link the test against
the corresponding requirement. On how-to do that see the
[requirement linking section](#how-to-link-a-requirement-against-a-test).

### Obtaining the logging output
For logging we highly rely on the ROS logging framework 
(for C++ see [here](http://wiki.ros.org/roscpp/Overview/Logging), 
for Python see [here](http://wiki.ros.org/rospy_tutorials/Tutorials/Logging)).  
However, when it comes to testing, obtaining the logging output is not always
as straight forward.

#### How-to obtain logging output for rostests
To obtain the logging output for [rostests](http://wiki.ros.org/rostest), we
suggest:
- to first build the system and the test you are interested in via `catkin_make`
and then,
- execute: `rostest my_package_name my_test_filename --text`.
(The `--text` flag sends the logging output to the screen as described in the
[ROS wiki](http://wiki.ros.org/rostest/Commandline).)
  
Sometimes when e.g. an exception hits, you may not see the complete logging
output. To avoid this issue you can force line buffering for the ROS loggers,
by setting the corresponding environment variable to 1:
```
export ROSCONSOLE_STDOUT_LINE_BUFFERED=1
```  
For more information see the
[ROS wiki](http://wiki.ros.org/rosconsole#Force_line_buffering_for_ROS_logger).

#### How-to change logging level for a package
Sometimes it is necessary to change the logging level for some or all packages.
Both kind of modifications can be done by providing a logging config file. 
On details, how to do it, see the
[ROS wiki](http://wiki.ros.org/rosconsole#Configuration).

### How-to write proper tests
Although it sounds like a rather easy job, writing good tests is quite hard. During
the development, we came up with a number of rules which (we think) improve the
quality of tests.

#### Rule: *Each test should only test one objective*  
Testing more than one objective makes a test more difficult to 
understand. Why?
- Each additional objective increases the number of requirements which have
to be met by the test, in order, to test all objectives.
- The order in which the objectives are tested might become important, although
this should have no relevance.
- Each additional objective increases the number of lines of code.
- If you do not believe us, then maybe the Google team can convince you,
therefore, see the
[google testing blog](https://testing.googleblog.com/2018/06/testing-on-toilet-keep-tests-focused.html)
  
#### Rule: *Keep tests as short as possible*  
A "small" test increases the understandability and decreases the hurdle
to refactor a test if needed.
  
#### Rule: *Use gtests instead of rostests for unit testing*  
We discovered that often it seems easier to use a rostest to perform
an actual unit-test. However, the decision to use a rostest, comes with
some heavy costs:
- For each rostest a roscore (+ ros nodes) is started at the beginning of
each test and shutdown at the end of each test. This consumes a
lot of time. If you have only one or two tests, everything is fine.
However, if you add more and more tests, the time you need to start and shutdown
your tests will become a significant time factor. This, in turn, decreases
the usefulness of the tests because the time needed to get feedback from the
tests is an important factor.
- Using a rostest means that you use the standard ROS mechanisms
(like topics, services, etc.) for communication. This leads to
threading which increases the complexity of your tests and, therefore,
increases the chance that your tests will be flaky.

If you reach a point in your development where you want to use a rostest
to implement an unit-test, try to take a step back and ask yourself:
"Why can I not use a gtest for a simple unit-test?" Most of the
time you will discover that you neglected the testability of your
code. If you have a certain problem, which makes you believe you need a 
rostest, then please have a look at our unit-tests, maybe they can give you
some inspirations how to solve these problems.

#### Rule: *Avoid threading whenever possible*
Whenever possible we suggest that you avoid threading. If you have an
integration or acceptance test, it cannot be avoided. However, as long as
you are dealing with unit tests, we strongly recommend that you use pure
gtests (in combination with gmock if necessary). Gtests not only run faster
but they allow you to avoid threading. Threading errors can be quite tricky 
because they are difficult to understand and track (especially if they
are sporadic).  
If you have to write a test which needs threading, have a look at section:
[How-to deal with threads in tests](#how-to-deal-with-threads-in-tests)


### How-to deal with threads in tests
As soon as you are dealing with rostests you are dealing with threads.
Even if you are not directly using threads, the ROS communication system
(topics, services, actions, etc.) does. In our experience, tests with
more than one thread are often much more flaky.  
To avoid flaky tests, in case of threading, some important rules need to be 
kept in mind.

#### Rule: *Use AsyncSpinner for service calls in test*  
If you are somehow using/calling services in your tests, don't forget to start
an [AsyncSpinner](http://docs.ros.org/melodic/api/roscpp/html/classros_1_1AsyncSpinner.html).
Otherwise, your service calls will not be processed and 
your test gets stuck. For more information about callbacks and spinning have
a look at the [ROS wiki](http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning).  
You can start an AsyncSpinner by adding the following
lines to your test main function:
```
ros::AsyncSpinner spinner{1};
spinner.start();
```

#### Rule: *Never use `sleep`*
What ever you do, do *never ever ever* use `sleep` in a test to wait for a
certain event. Why not?  
Let's assume that you state a certain sleep duration to wait for a certain
event in your test. Let's even assume that the test always runs
successfully on your particular hardware. Who guarantees you that a 
different hardware setup (like on the CI servers) does not lead to a different
timing behavior and, therefore, to potential errors?  
These kind of errors are particular hard to understand and track,
because they normally are highly sporadic and you might not even be able 
to reproduce the error.  
  
We often dealt with these kind of problems and came up with a helper class
called [`AsyncTest`](https://github.com/PilzDE/pilz_robots/blob/melodic-devel/pilz_testutils/include/pilz_testutils/async_test.h). The `AsyncTest` class allows you to easily
signal that a certain event (like a topic or service
call) has happend and of course to wait for a certain signal to happen.
To see how this helper class can be used, have a look at our 
[documentation](https://github.com/PilzDE/pilz_robots/blob/melodic-devel/pilz_testutils/include/pilz_testutils/async_test.h#L35).

#### Rule: *Wait until nodes, topics and services are up and running*
When using a rostest, you have to ensure that your system is completely up
and running, before you start your actual tests. It normally takes a 
while before all nodes and their topics, services, actions, etc. are up 
and running. If you do not account for this effect, 
you might encounter (sporadic) errors. Please be aware of the fact that 
these errors are especially difficult to understand and track, because they
might depend on the underlying hardware setup (e.g. slow vs. fast PC, etc.).  
  
We implemented some helper functions which might be useful:
- wait till a certain node has started -> see function
[waitForNode](https://github.com/PilzDE/pilz_robots/blob/melodic-devel/prbt_hardware_support/test/include/prbt_hardware_support/ros_test_helper.h#L47)
- wait till service is up and running -> see function
[waitForService](https://github.com/PilzDE/pilz_robots/blob/melodic-devel/prbt_hardware_support/include/prbt_hardware_support/wait_for_service.h#L34)
- wait till topic is up and running -> see function
[waitForTopic](https://github.com/PilzDE/pilz_robots/blob/melodic-devel/prbt_hardware_support/include/prbt_hardware_support/wait_for_topic.h#L35)


### Gtest command line flags
Gtests are ordinary executables which come with a number
of build in command line flags. These flags can be quite helpful
and we strongly encourage you to make use of them. You can find a description
of the different command line flags in the 
[googletest github repository](https://github.com/google/googletest/blob/master/googletest/docs/advanced.md#running-test-programs-advanced-options).

**Note:**  
Given that you are building with `catkin_make`, you usually can find the 
gtest executable in your workspace under
`/path_to_my_ros_workspace/devel/lib/my_package/`. 

### How-to run particular test of test suite
If you only want to run a certain test of a test suite, set the 
environment variable `GTEST_FILTER`. For example:  
If you only want to run the test `my_awesome_test` then set
the environment variable in your terminal as follows:
```
export GTEST_FILTER=*my_awesome_test*
```
If you are done and want again run all your tests then don't forget to reset
the `GTEST_FILTER` variable by setting:
```
export GTEST_FILTER=*
```

## Mocking the hardware
TODO

## FAQ
- **Q:** *How can I install missing system dependencies for a package?*  
  **A:** To install the missing system dependencies for a package you can use:  
  `rosdep install --from-paths WORKSPACE --ignore-src --rosdistro=ROSDISTRO -y`  
Replace `WORKSPACE` with the path to your ROS workspace e.g. `~/catkin_ws/`
and `ROSDISTRO` with e.g. `melodic` (or `kinetic` respectively).  
Instead of `-y` which confirms all actions automatically, you can also 
use `--simulate` which shows you what `rosdep` wants to do. For more
information about `rosdep` see the [ROS wiki](http://wiki.ros.org/rosdep)
and also [docs.ros.org](https://docs.ros.org/independent/api/rosdep/html/).
