#ifndef CNOID_IRSL_SIMSTEP_CONTROLLER_ITEM_H
#define CNOID_IRSL_SIMSTEP_CONTROLLER_ITEM_H

#include <cnoid/ControllerItem>
//#include "exportdecl.h"
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <iostream>

namespace cnoid {

#define __CNOID_EXPORT __attribute__ ((visibility("default")))

class __CNOID_EXPORT SimStepControllerItem : public ControllerItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    SimStepControllerItem();
    SimStepControllerItem(const SimStepControllerItem& org);
    virtual ~SimStepControllerItem();

    virtual bool checkIfSubController(ControllerItem* controllerItem) const override;
    virtual bool initialize(ControllerIO* io) override;
    virtual bool start() override;
    virtual double timeStep() const override;
    virtual void input() override;
    virtual bool control() override;
    virtual void output() override;
    virtual void stop() override;

    bool check_sim_step() {
        bool ret = mu.try_lock();
        if (ret) mu.unlock();
        return ret;
    }
    // check_lock -> false / wait until true
    // check_lock -> true / process after that notify()
    long wait_next_step() {
        long ret;
        mu.lock();
        ret = wait_counter;
        mu.unlock();
        return ret;
    }

    void notify_sim_step() {
        cond.wakeAll();
    }

    void wait_control_notify() {
        wait_counter++;
        cond.wait(&mu); // mu(release) => waiting
    }

    void wait_steps(int nstep) {
        wait_next_step();
        long target = wait_counter + nstep;
        while(wait_counter >= target) {
            notify_sim_step();
            wait_next_step();
        }
    }

    void disable_stepping() {
        started = false;
        notify_sim_step();
    }
    void enable_stepping() {
        started = true;
    }

    void stop_on_output(bool on_) {
        stop_on_output_ = on_;
    }
    bool at_output() {  return at_output_; }

    void usleep(unsigned long usec) {  QThread::usleep(usec); }

    double currentTime() const;

    Body *getSimBody() {
        if (!simulationBody) {
#ifdef _IRSL_DEBUG_
            std::cerr << "body is empty" << std::endl;
#endif
        }
        return simulationBody;
    }

private:
    // access to io->Body ...
    Body* simulationBody;
    ControllerIO* io;
    bool started;
    bool stop_on_output_;
    bool at_output_;
protected:
    virtual Item* doDuplicate() const override;
    virtual void onTreePathChanged() override;
    virtual void onDisconnectedFromRoot() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    long wait_counter;
    QMutex mu;
    QWaitCondition cond;
};

typedef ref_ptr<SimStepControllerItem> SimStepControllerItemPtr;

}

#endif
