package com.example.taxi.ui.mypage.boarding_list

import android.util.Log
import android.view.View
import androidx.core.os.bundleOf
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.user.boarded_taxi_list.BoardedTaxi
import com.example.taxi.data.dto.user.boarded_taxi_list.BoardedTaxiList
import com.example.taxi.databinding.FragmentBoardingListBinding
import com.example.taxi.ui.driving.end.EndDrivingViewModel
import com.example.taxi.ui.mypage.MyPageAdapter
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class BoardingListFragment : BaseFragment<FragmentBoardingListBinding>(R.layout.fragment_boarding_list) {
    private val endDrivingViewModel : EndDrivingViewModel by viewModels()

    private lateinit var boardingListAdapter: BoardingListAdapter
    private lateinit var boardedTaxiList: BoardedTaxiList

    private val onClickListener: (View, Int) -> Unit = { _, idx ->
        findNavController().navigate(R.id.action_boardingListFragment_to_taxiDetailFragment, bundleOf("BoardedTaxi" to boardedTaxiList.taxiList[idx], "index" to idx))
    }

    override fun init() {
        initData()
        observerData()
    }
    private fun initData() {
        endDrivingViewModel.getBoardedTaxiList()
    }

    private fun observerData() {
        endDrivingViewModel.boardedTaxiList.observe(viewLifecycleOwner) { state ->
            when (state) {
                is UiState.Loading -> {
                    //binding.progressBar.show()
                }
                is UiState.Failure -> {
                    //binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                }
                is UiState.Success -> {
                    //binding.progressBar.hide()
//                    boardedTaxiList = state.data.taxiList as MutableList<BoardedTaxi>
                    boardedTaxiList = state.data
                    initAdapter(boardedTaxiList)
                }
            }
        }
    }

    private fun initAdapter(TaxiList: BoardedTaxiList) {
        boardingListAdapter = BoardingListAdapter().apply {
            onBoaredTaxiClickListener = onClickListener
            context = requireContext()
            boardedTaxiList = TaxiList.taxiList as MutableList<BoardedTaxi>
        }
        binding.recyclerviewMyPageBoardTaxiList.apply {
            adapter = boardingListAdapter
            layoutManager = LinearLayoutManager(requireContext(), RecyclerView.VERTICAL, false)
        }
    }
}