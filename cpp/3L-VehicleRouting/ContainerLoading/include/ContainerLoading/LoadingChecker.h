#pragma once

#include "CommonBasics/Helper/ModelServices.h"

#include "ProblemParameters.h"

#include "Algorithms/MultiContainer/BP_MIP_1D.h"
#include "Model/ContainerLoadingInstance.h"

#include <boost/dynamic_bitset.hpp>
#include <boost/functional/hash.hpp>

#include <chrono>

namespace ContainerLoading
{
using namespace Algorithms;

class LoadingChecker
{
  public:
    const ContainerLoadingParams Parameters;

    explicit LoadingChecker(const ContainerLoadingParams& parameters) : Parameters(parameters)
    {
        using enum LoadingFlag;

        std::vector<LoadingFlag> usedLoadingFlags = {Complete, NoSupport, LifoNoSequence};

        constexpr size_t reservedSize = 1000;
        for (const auto flag: usedLoadingFlags)
        {
            mFeasSequences[flag & Parameters.LoadingProblem.LoadingFlags].reserve(reservedSize);
            mInfSequences[flag & Parameters.LoadingProblem.LoadingFlags].reserve(reservedSize);
            mUnkSequences[flag & Parameters.LoadingProblem.LoadingFlags].reserve(reservedSize);

            mFeasibleSets[flag & Parameters.LoadingProblem.LoadingFlags].reserve(reservedSize);
            mInfSets[flag & Parameters.LoadingProblem.LoadingFlags].reserve(reservedSize);
            mUnknownSets[flag & Parameters.LoadingProblem.LoadingFlags].reserve(reservedSize);
        }

        mStartTime = std::chrono::high_resolution_clock::now();
    }

    [[nodiscard]] std::vector<Cuboid>
        SelectItems(const Collections::IdVector& nodeIds, std::vector<Group>& nodes, bool reversedDirection) const;

    [[nodiscard]] LoadingStatus PackingHeuristic(PackingType packingType,
                                                 const Container& container,
                                                 const Collections::IdVector& stopIds,
                                                 const std::vector<Cuboid>& items);

    [[nodiscard]] LoadingStatus ConstraintProgrammingSolver(PackingType packingType,
                                                            const Container& container,
                                                            const boost::dynamic_bitset<>& set,
                                                            const Collections::IdVector& stopIds,
                                                            const std::vector<Cuboid>& items,
                                                            bool isCallTypeExact,
                                                            double maxRuntime = std::numeric_limits<double>::max());

    [[nodiscard]] LoadingStatus ConstraintProgrammingSolverGetPacking(PackingType packingType,
                                                                      const Container& container,
                                                                      const Collections::IdVector& stopIds,
                                                                      std::vector<Cuboid>& items,
                                                                      double maxRuntime) const;

    [[nodiscard]] LoadingStatus HeuristicCompleteCheck(const Container& container,
                                                       const boost::dynamic_bitset<>& set,
                                                       const Collections::IdVector& stopIds,
                                                       const std::vector<Cuboid>& items,
                                                       double maxRuntime = std::numeric_limits<double>::max());

    void SetBinPackingModel(GRBEnv* env,
                            std::vector<Container>& containers,
                            std::vector<Group>& nodes,
                            const std::string& outputPath = "");

    [[nodiscard]] int SolveBinPackingApproximation() const;

    [[nodiscard]] int DetermineMinVehicles(bool enableLifting,
                                           double liftingThreshold,
                                           const Container& container,
                                           const boost::dynamic_bitset<>& nodes,
                                           double weight,
                                           double volume) const;

    [[nodiscard]] bool CustomerCombinationInfeasible(const boost::dynamic_bitset<>& customersInRoute) const;
    void AddInfeasibleCombination(const boost::dynamic_bitset<>& customersInRoute);
    [[nodiscard]] double GetElapsedTime();

    [[nodiscard]] Collections::SequenceVector GetFeasibleRoutes() const;
    [[nodiscard]] size_t GetNumberOfFeasibleRoutes() const;
    [[nodiscard]] size_t GetSizeInfeasibleCombinations() const;

    void AddFeasibleSequenceMask(const Collections::IdVector& route);

    [[nodiscard]] bool RouteIsInFeasSequences(const Collections::IdVector& route) const;

    void AddSequenceCheckedTwoOpt(const Collections::IdVector& sequence);

    [[nodiscard]] bool SequenceIsCheckedTwoOpt(const Collections::IdVector& sequence) const;

    [[nodiscard]] boost::dynamic_bitset<> MakeBitset(size_t size, const Collections::IdVector& sequence) const;

    [[nodiscard]] std::unordered_map<double, Collections::IdVector> GetFeasibleRoutesWithTimeStamps()
    {
        return mCompleteFeasSeqWithTimeStamps;
    };

    [[nodiscard]] std::unordered_map<double, Collections::IdVector> GetInfeasibleRoutesWithTimeStamps()
    {
        return mCompleteInfeasSeqWithTimeStamps;
    };

    [[nodiscard]] std::unordered_map<double, Collections::IdVector> GetUnknownRoutesWithTimeStamps()
    {
        return mCompleteUnknownSeqWithTimeStamps;
    };

    [[nodiscard]] std::unordered_map<double, Collections::IdVector> GetInvalidRoutesWithTimeStamps()
    {
        return mCompleteInvalidSeqWithTimeStamps;
    };

    void AddTailTournamentConstraint(const Collections::IdVector& sequence)
    {
        auto elapsed = GetElapsedTime();
        mTailTournamentConstraintsWithTimeStamps.insert({elapsed, sequence});
    }
    [[nodiscard]] std::unordered_map<double, Collections::IdVector> GetTailTournamentConstraints() const
    {
        return mTailTournamentConstraintsWithTimeStamps;
    }

    // TODO Possible reallocate back to private 
    void AddInfeasibleRoute(const Collections::IdVector& route);
    void AddFeasibleRoute(const Collections::IdVector& route);
    void AddUnknownRoute(const Collections::IdVector& route);
    void AddInvalidRoute(const Collections::IdVector& route);


    [[nodiscard]] bool RouteIsInFeasSequences(const Collections::IdVector& route,
                                              LoadingFlag mask) const
    {
        return mFeasSequences.at(mask).contains(route);
    }

    [[nodiscard]] bool RouteIsInInfSequences(const Collections::IdVector& route,
                                             LoadingFlag mask) const
    {
        return mInfSequences.at(mask).contains(route);
    }

    [[nodiscard]] bool RouteIsInUnkSequences(const Collections::IdVector& route,
                                             LoadingFlag mask) const
    {
        return mUnkSequences.at(mask).contains(route);
    }

  private:
    std::chrono::high_resolution_clock::time_point mStartTime;
    std::unique_ptr<BinPacking1D> mBinPacking1D;

    Collections::SequenceSet mTwoOptCheckedSequences;

    Collections::SequenceSet mEPHeurInfSequences;
    Collections::SequenceVector mCompleteFeasSeq;
    std::unordered_map<double, Collections::IdVector> mCompleteFeasSeqWithTimeStamps;
    std::unordered_map<double, Collections::IdVector> mCompleteInfeasSeqWithTimeStamps;
    std::unordered_map<double, Collections::IdVector> mCompleteUnknownSeqWithTimeStamps;
    std::unordered_map<double, Collections::IdVector> mCompleteInvalidSeqWithTimeStamps;
    std::unordered_map<double, Collections::IdVector> mTailTournamentConstraintsWithTimeStamps;

    /// Set of customer combinations that are infeasible.
    /// -> There is no path in combination C that respects all constraints
    /// -> At least 2 vehicles are needed to serve all customers in C
    std::vector<boost::dynamic_bitset<>> mInfeasibleCustomerCombinations;

    std::unordered_map<LoadingFlag, std::vector<boost::dynamic_bitset<>>> mFeasibleSets;
    std::unordered_map<LoadingFlag, Collections::SequenceSet> mFeasSequences;

    std::unordered_map<LoadingFlag, std::vector<boost::dynamic_bitset<>>> mInfSets;
    std::unordered_map<LoadingFlag, Collections::SequenceSet> mInfSequences;

    std::unordered_map<LoadingFlag, std::vector<boost::dynamic_bitset<>>> mUnknownSets;
    std::unordered_map<LoadingFlag, Collections::SequenceSet> mUnkSequences;

    [[nodiscard]] bool SequenceIsHeuristicallyInfeasibleEP(const Collections::IdVector& sequence) const;
    void AddInfeasibleSequenceEP(const Collections::IdVector& sequence);

    [[nodiscard]] bool SequenceIsInfeasibleCP(const Collections::IdVector& sequence, LoadingFlag mask) const;
    [[nodiscard]] bool SequenceIsUnknownCP(const Collections::IdVector& sequence, LoadingFlag mask) const;
    [[nodiscard]] bool SequenceIsFeasible(const Collections::IdVector& sequence, LoadingFlag mask) const;

    [[nodiscard]] bool SetIsInfeasibleCP(const boost::dynamic_bitset<>& set, LoadingFlag mask) const;
    [[nodiscard]] bool SetIsUnknownCP(const boost::dynamic_bitset<>& set, LoadingFlag mask) const;
    [[nodiscard]] bool SetIsFeasibleCP(const boost::dynamic_bitset<>& set, LoadingFlag mask) const;

    [[nodiscard]] LoadingFlag BuildMask(PackingType type) const;

    [[nodiscard]] LoadingStatus GetPrecheckStatusCP(const Collections::IdVector& sequence,
                                                    const boost::dynamic_bitset<>& set,
                                                    LoadingFlag mask,
                                                    bool isCallTypeExact);

    void AddStatus(const Collections::IdVector& sequence,
                   const boost::dynamic_bitset<>& set,
                   LoadingFlag mask,
                   LoadingStatus status);

    [[nodiscard]] LoadingStatus RunLoadingHeuristic(PackingType packingType,
                                                    const Container& container [[maybe_unused]],
                                                    const Collections::IdVector& stopIds [[maybe_unused]],
                                                    const std::vector<Cuboid>& items [[maybe_unused]]);

    [[nodiscard]] int DetermineMinVehiclesBinPacking(bool enableLifting,
                                                     double liftingThreshold,
                                                     const boost::dynamic_bitset<>& nodes,
                                                     int r,
                                                     double z) const;

    [[nodiscard]] int ReSolveBinPackingApproximation(const boost::dynamic_bitset<>& selectedGroups) const;
};

}